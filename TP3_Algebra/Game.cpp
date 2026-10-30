#include "Game.h"

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "raylib.h"
#include "Geometry.h"
#include "Polyhedron.h"
#include "CollisionGrid.h"
#include "Rendering.h"

namespace ReyesCuadra
{
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    const float GRID_SIDE_LENGTH = 20.0f;
    const float GRID_SPACING_OPTIONS[] = { 0.05f, 0.1f, 0.2f, 0.5f };
    const int GRID_SPACING_OPTION_COUNT = 4;
    const int DEFAULT_GRID_SPACING_OPTION = 1;

    const float POSITION_LIMIT_MARGIN = 2.0f;

    const float MOVE_SPEED = 5.0f;
    const float ROTATION_SPEED = 90.0f;
    const float SCALE_SPEED = 1.0f;
    const float MIN_SCALE = 0.2f;
    const float MAX_SCALE = 4.0f;

    const int MAX_SHARED_POINTS_TO_DRAW = 4000;

    const float CAMERA_ROTATION_SENSITIVITY = 0.005f;
    const float CAMERA_ZOOM_STEP = 1.0f;
    const float CAMERA_MIN_DISTANCE = 3.0f;
    const float CAMERA_MAX_DISTANCE = 45.0f;
    const float CAMERA_MAX_PITCH = 1.5f;

    static const Color BACKGROUND_COLOR = { 24, 26, 32, 255 };
    static const Color GRID_BOUNDS_COLOR = { 80, 80, 95, 255 };
    static const Color BOX_FREE_COLOR = { 90, 220, 120, 255 };
    static const Color BOX_OVERLAP_COLOR = { 255, 160, 40, 255 };
    static const Color BOX_COLLIDING_COLOR = { 240, 70, 70, 255 };
    static const Color OVERLAP_ZONE_COLOR = { 255, 230, 80, 255 };
    static const Color SHARED_POINT_COLOR = { 255, 80, 255, 255 };
    static const Color HUD_TEXT_COLOR = { 230, 230, 230, 255 };
    static const Color HUD_PANEL_COLOR = { 0, 0, 0, 150 };

    struct ModelDefinition
    {
        const char* fileName;
        const char* displayName;
        Vector3 startPosition;
        Color baseColor;
    };

    const ModelDefinition MODEL_DEFINITIONS[] =
    {
        { "cube.obj",         "Cubo",       { -7.5f,  0.0f, 0.0f }, {  70, 130, 220, 255 } },
        { "tetrahedron.obj",  "Tetraedro",  { -4.5f, -0.4f, 0.0f }, {  80, 200, 160, 255 } },
        { "octahedron.obj",   "Octaedro",   { -1.5f,  0.0f, 0.0f }, { 200, 170,  70, 255 } },
        { "decahedron.obj",   "Decaedro",   {  1.5f, -1.0f, 0.0f }, { 170, 110, 220, 255 } },
        { "dodecahedron.obj", "Dodecaedro", {  4.5f,  0.0f, 0.0f }, {  90, 190, 230, 255 } },
        { "icosahedron.obj",  "Icosaedro",  {  7.5f,  0.0f, 0.0f }, { 230, 140, 100, 255 } }
    };
    const int MODEL_COUNT = 6;

    struct OrbitCamera
    {
        Camera3D camera;
        float yawRadians;
        float pitchRadians;
        float distanceToTarget;
    };

    struct PairCollision
    {
        int firstIndex;
        int secondIndex;
        CollisionResult result;
    };

    struct GameState
    {
        std::vector<Polyhedron> polyhedra;
        std::vector<bool> isBoxOverlapping;
        std::vector<PairCollision> pairCollisions;
        std::vector<Vector3> sharedPointsToDraw;
        std::vector<std::string> missingFiles;

        CollisionGrid collisionGrid;
        int gridSpacingOption;

        int selectedIndex;
        OrbitCamera orbitCamera;

        bool showBoundingBoxes;
        bool showNormals;
        bool showSharedPoints;
    };

    static std::string FindModelPath(const std::string& fileName)
    {
        const char* searchFolders[] =
        {
            "res/models/", "../res/models/", "../../res/models/", "../../../res/models/", "../../../../res/models/"
        };

        for (const char* searchFolder : searchFolders)
        {
            std::string candidatePath = std::string(searchFolder) + fileName;
            std::ifstream candidateFile(candidatePath);

            if (candidateFile.is_open())
            {
                return candidatePath;
            }
        }

        return "";
    }

    static void LoadAllModels(GameState& gameState)
    {
        for (int modelIndex = 0; modelIndex < MODEL_COUNT; modelIndex++)
        {
            const ModelDefinition& definition = MODEL_DEFINITIONS[modelIndex];

            Polyhedron newPolyhedron;
            std::string modelPath = FindModelPath(definition.fileName);

            if (modelPath.empty() || !LoadPolyhedronFromObj(modelPath, newPolyhedron))
            {
                gameState.missingFiles.push_back(definition.fileName);
                continue;
            }

            newPolyhedron.displayName = definition.displayName;
            newPolyhedron.position = definition.startPosition;
            newPolyhedron.baseColor = definition.baseColor;

            gameState.polyhedra.push_back(newPolyhedron);
        }

        gameState.isBoxOverlapping.assign(gameState.polyhedra.size(), false);
    }

    static void UpdateOrbitCameraPosition(OrbitCamera& orbitCamera)
    {
        float horizontalDistance = std::cos(orbitCamera.pitchRadians) * orbitCamera.distanceToTarget;

        Vector3 offsetFromTarget =
        {
            horizontalDistance * std::sin(orbitCamera.yawRadians),
            std::sin(orbitCamera.pitchRadians) * orbitCamera.distanceToTarget,
            horizontalDistance * std::cos(orbitCamera.yawRadians)
        };

        orbitCamera.camera.position = AddVectors(orbitCamera.camera.target, offsetFromTarget);
    }

    static void InitOrbitCamera(OrbitCamera& orbitCamera)
    {
        orbitCamera.camera.target = { 0.0f, 0.0f, 0.0f };
        orbitCamera.camera.up = { 0.0f, 1.0f, 0.0f };
        orbitCamera.camera.fovy = 45.0f;
        orbitCamera.camera.projection = CAMERA_PERSPECTIVE;

        orbitCamera.yawRadians = 0.5f;
        orbitCamera.pitchRadians = 0.55f;
        orbitCamera.distanceToTarget = 20.0f;

        UpdateOrbitCameraPosition(orbitCamera);
    }

    static void HandleCameraInput(OrbitCamera& orbitCamera)
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
        {
            Vector2 mouseDelta = GetMouseDelta();
            orbitCamera.yawRadians -= mouseDelta.x * CAMERA_ROTATION_SENSITIVITY;
            orbitCamera.pitchRadians += mouseDelta.y * CAMERA_ROTATION_SENSITIVITY;

            if (orbitCamera.pitchRadians > CAMERA_MAX_PITCH) orbitCamera.pitchRadians = CAMERA_MAX_PITCH;
            if (orbitCamera.pitchRadians < -CAMERA_MAX_PITCH) orbitCamera.pitchRadians = -CAMERA_MAX_PITCH;
        }

        orbitCamera.distanceToTarget -= GetMouseWheelMove() * CAMERA_ZOOM_STEP;

        if (orbitCamera.distanceToTarget < CAMERA_MIN_DISTANCE) orbitCamera.distanceToTarget = CAMERA_MIN_DISTANCE;
        if (orbitCamera.distanceToTarget > CAMERA_MAX_DISTANCE) orbitCamera.distanceToTarget = CAMERA_MAX_DISTANCE;

        UpdateOrbitCameraPosition(orbitCamera);
    }

    static float ClampValue(float value, float minValue, float maxValue)
    {
        if (value < minValue) return minValue;
        if (value > maxValue) return maxValue;
        return value;
    }

    static float WrapDegrees(float angleDegrees)
    {
        float wrappedAngle = std::fmod(angleDegrees, 360.0f);
        return wrappedAngle < 0.0f ? wrappedAngle + 360.0f : wrappedAngle;
    }

    static void HandleSelectedObjectInput(GameState& gameState, float deltaTime)
    {
        if (gameState.polyhedra.empty())
        {
            return;
        }

        int polyhedronCount = static_cast<int>(gameState.polyhedra.size());

        if (IsKeyPressed(KEY_TAB))
        {
            int stepDirection = IsKeyDown(KEY_LEFT_SHIFT) ? -1 : 1;
            gameState.selectedIndex = (gameState.selectedIndex + stepDirection + polyhedronCount) % polyhedronCount;
        }

        Polyhedron& selectedPolyhedron = gameState.polyhedra[gameState.selectedIndex];

        float moveAmount = MOVE_SPEED * deltaTime;

        if (IsKeyDown(KEY_D)) selectedPolyhedron.position.x += moveAmount;
        if (IsKeyDown(KEY_A)) selectedPolyhedron.position.x -= moveAmount;
        if (IsKeyDown(KEY_E)) selectedPolyhedron.position.y += moveAmount;
        if (IsKeyDown(KEY_Q)) selectedPolyhedron.position.y -= moveAmount;
        if (IsKeyDown(KEY_S)) selectedPolyhedron.position.z += moveAmount;
        if (IsKeyDown(KEY_W)) selectedPolyhedron.position.z -= moveAmount;

        AxisAlignedBox gridBounds = GetGridBounds(gameState.collisionGrid);

        if (gameState.collisionGrid.pointsPerAxis > 1)
        {
            selectedPolyhedron.position.x = ClampValue(selectedPolyhedron.position.x, gridBounds.minCorner.x + POSITION_LIMIT_MARGIN, gridBounds.maxCorner.x - POSITION_LIMIT_MARGIN);
            selectedPolyhedron.position.y = ClampValue(selectedPolyhedron.position.y, gridBounds.minCorner.y + POSITION_LIMIT_MARGIN, gridBounds.maxCorner.y - POSITION_LIMIT_MARGIN);
            selectedPolyhedron.position.z = ClampValue(selectedPolyhedron.position.z, gridBounds.minCorner.z + POSITION_LIMIT_MARGIN, gridBounds.maxCorner.z - POSITION_LIMIT_MARGIN);
        }

        float rotationAmount = ROTATION_SPEED * deltaTime;

        if (IsKeyDown(KEY_I)) selectedPolyhedron.rotationDegrees.x += rotationAmount;
        if (IsKeyDown(KEY_K)) selectedPolyhedron.rotationDegrees.x -= rotationAmount;
        if (IsKeyDown(KEY_L)) selectedPolyhedron.rotationDegrees.y += rotationAmount;
        if (IsKeyDown(KEY_J)) selectedPolyhedron.rotationDegrees.y -= rotationAmount;
        if (IsKeyDown(KEY_O)) selectedPolyhedron.rotationDegrees.z += rotationAmount;
        if (IsKeyDown(KEY_U)) selectedPolyhedron.rotationDegrees.z -= rotationAmount;

        selectedPolyhedron.rotationDegrees.x = WrapDegrees(selectedPolyhedron.rotationDegrees.x);
        selectedPolyhedron.rotationDegrees.y = WrapDegrees(selectedPolyhedron.rotationDegrees.y);
        selectedPolyhedron.rotationDegrees.z = WrapDegrees(selectedPolyhedron.rotationDegrees.z);

        float scaleChange = 0.0f;
        if (IsKeyDown(KEY_R)) scaleChange += SCALE_SPEED * deltaTime;
        if (IsKeyDown(KEY_F)) scaleChange -= SCALE_SPEED * deltaTime;

        if (scaleChange != 0.0f)
        {
            bool onlyAxisX = IsKeyDown(KEY_ONE);
            bool onlyAxisY = IsKeyDown(KEY_TWO);
            bool onlyAxisZ = IsKeyDown(KEY_THREE);
            bool scaleAllAxes = !onlyAxisX && !onlyAxisY && !onlyAxisZ;

            if (scaleAllAxes || onlyAxisX) selectedPolyhedron.scale.x = ClampValue(selectedPolyhedron.scale.x + scaleChange, MIN_SCALE, MAX_SCALE);
            if (scaleAllAxes || onlyAxisY) selectedPolyhedron.scale.y = ClampValue(selectedPolyhedron.scale.y + scaleChange, MIN_SCALE, MAX_SCALE);
            if (scaleAllAxes || onlyAxisZ) selectedPolyhedron.scale.z = ClampValue(selectedPolyhedron.scale.z + scaleChange, MIN_SCALE, MAX_SCALE);
        }

        if (IsKeyPressed(KEY_BACKSPACE))
        {
            for (int modelIndex = 0; modelIndex < MODEL_COUNT; modelIndex++)
            {
                if (selectedPolyhedron.displayName == MODEL_DEFINITIONS[modelIndex].displayName)
                {
                    selectedPolyhedron.position = MODEL_DEFINITIONS[modelIndex].startPosition;
                }
            }

            selectedPolyhedron.rotationDegrees = { 0.0f, 0.0f, 0.0f };
            selectedPolyhedron.scale = { 1.0f, 1.0f, 1.0f };
        }
    }

    static void HandleOptionsInput(GameState& gameState)
    {
        if (IsKeyPressed(KEY_B)) gameState.showBoundingBoxes = !gameState.showBoundingBoxes;
        if (IsKeyPressed(KEY_N)) gameState.showNormals = !gameState.showNormals;
        if (IsKeyPressed(KEY_G)) gameState.showSharedPoints = !gameState.showSharedPoints;

        int previousOption = gameState.gridSpacingOption;

        if (IsKeyPressed(KEY_F1) && gameState.gridSpacingOption > 0) gameState.gridSpacingOption--;
        if (IsKeyPressed(KEY_F2) && gameState.gridSpacingOption < GRID_SPACING_OPTION_COUNT - 1) gameState.gridSpacingOption++;

        if (previousOption != gameState.gridSpacingOption)
        {
            float newSpacing = GRID_SPACING_OPTIONS[gameState.gridSpacingOption];
            gameState.collisionGrid = CreateCollisionGrid({ 0.0f, 0.0f, 0.0f }, GRID_SIDE_LENGTH, newSpacing);
        }
    }

    static void UpdateCollisions(GameState& gameState)
    {
        int polyhedronCount = static_cast<int>(gameState.polyhedra.size());

        for (Polyhedron& polyhedron : gameState.polyhedra)
        {
            UpdatePolyhedronWorldData(polyhedron);
            polyhedron.isColliding = false;
        }

        gameState.isBoxOverlapping.assign(polyhedronCount, false);
        gameState.pairCollisions.clear();

        for (int firstIndex = 0; firstIndex < polyhedronCount; firstIndex++)
        {
            for (int secondIndex = firstIndex + 1; secondIndex < polyhedronCount; secondIndex++)
            {
                Polyhedron& firstPolyhedron = gameState.polyhedra[firstIndex];
                Polyhedron& secondPolyhedron = gameState.polyhedra[secondIndex];

                CollisionResult collisionResult = CheckPolyhedronCollision(firstPolyhedron, secondPolyhedron, gameState.collisionGrid);

                if (!collisionResult.areBoxesOverlapping)
                {
                    continue;
                }

                gameState.isBoxOverlapping[firstIndex] = true;
                gameState.isBoxOverlapping[secondIndex] = true;

                if (collisionResult.areModelsColliding)
                {
                    firstPolyhedron.isColliding = true;
                    secondPolyhedron.isColliding = true;
                }

                gameState.pairCollisions.push_back({ firstIndex, secondIndex, collisionResult });
            }
        }

        gameState.sharedPointsToDraw.clear();

        if (gameState.showSharedPoints)
        {
            for (const PairCollision& pairCollision : gameState.pairCollisions)
            {
                if (!pairCollision.result.areModelsColliding)
                {
                    continue;
                }

                if (static_cast<int>(gameState.sharedPointsToDraw.size()) >= MAX_SHARED_POINTS_TO_DRAW)
                {
                    break;
                }

                CollectSharedGridPoints(gameState.polyhedra[pairCollision.firstIndex], gameState.polyhedra[pairCollision.secondIndex],
                                        gameState.collisionGrid, gameState.sharedPointsToDraw, MAX_SHARED_POINTS_TO_DRAW);
            }
        }
    }

    static void DrawWorldAxes()
    {
        Vector3 origin = { 0.0f, 0.0f, 0.0f };
        DrawLine3D(origin, { 1.5f, 0.0f, 0.0f }, { 230, 70, 70, 255 });
        DrawLine3D(origin, { 0.0f, 1.5f, 0.0f }, { 70, 230, 70, 255 });
        DrawLine3D(origin, { 0.0f, 0.0f, 1.5f }, { 70, 120, 240, 255 });
    }

    static void DrawScene(const GameState& gameState)
    {
        BeginMode3D(gameState.orbitCamera.camera);

        DrawWorldAxes();

        DrawAxisAlignedBox(GetGridBounds(gameState.collisionGrid), GRID_BOUNDS_COLOR);

        int polyhedronCount = static_cast<int>(gameState.polyhedra.size());

        for (int polyhedronIndex = 0; polyhedronIndex < polyhedronCount; polyhedronIndex++)
        {
            const Polyhedron& polyhedron = gameState.polyhedra[polyhedronIndex];
            DrawPolyhedron(polyhedron, polyhedronIndex == gameState.selectedIndex);

            if (gameState.showNormals)
            {
                DrawPolyhedronNormals(polyhedron);
            }

            if (gameState.showBoundingBoxes)
            {
                Color boxColor = BOX_FREE_COLOR;
                if (gameState.isBoxOverlapping[polyhedronIndex]) boxColor = BOX_OVERLAP_COLOR;
                if (polyhedron.isColliding) boxColor = BOX_COLLIDING_COLOR;

                DrawAxisAlignedBox(polyhedron.boundingBox, boxColor);
            }
        }

        for (const PairCollision& pairCollision : gameState.pairCollisions)
        {
            if (gameState.showBoundingBoxes)
            {
                AxisAlignedBox overlapBox = GetBoxIntersection(gameState.polyhedra[pairCollision.firstIndex].boundingBox,
                                                               gameState.polyhedra[pairCollision.secondIndex].boundingBox);
                DrawAxisAlignedBox(overlapBox, OVERLAP_ZONE_COLOR);
            }

            if (pairCollision.result.areModelsColliding)
            {
                DrawCube(pairCollision.result.sharedGridPoint, 0.1f, 0.1f, 0.1f, WHITE);
            }
        }

        float sharedPointSize = gameState.collisionGrid.pointSpacing * 0.35f;

        for (const Vector3& sharedPoint : gameState.sharedPointsToDraw)
        {
            DrawCube(sharedPoint, sharedPointSize, sharedPointSize, sharedPointSize, SHARED_POINT_COLOR);
        }

        EndMode3D();
    }

    static void DrawCollisionStatus(const GameState& gameState)
    {
        bool isAnyPairColliding = false;

        for (const PairCollision& pairCollision : gameState.pairCollisions)
        {
            if (pairCollision.result.areModelsColliding)
            {
                isAnyPairColliding = true;
            }
        }

        const char* statusText = "SIN COLISION";
        Color statusColor = BOX_FREE_COLOR;

        if (isAnyPairColliding)
        {
            statusText = "HAY COLISION";
            statusColor = BOX_COLLIDING_COLOR;
        }
        else if (!gameState.pairCollisions.empty())
        {
            statusText = "SIN COLISION (bounding boxes superpuestas)";
            statusColor = BOX_OVERLAP_COLOR;
        }

        int statusFontSize = 30;
        int statusTextWidth = MeasureText(statusText, statusFontSize);

        DrawRectangle(10, 10, statusTextWidth + 30, 50, HUD_PANEL_COLOR);
        DrawRectangle(10, 10, 8, 50, statusColor);
        DrawText(statusText, 28, 20, statusFontSize, statusColor);

        int lineY = 72;
        int detailFontSize = 18;
        int panelHeight = 30 + static_cast<int>(gameState.pairCollisions.size()) * 22;
        if (gameState.pairCollisions.empty()) panelHeight = 52;

        DrawRectangle(10, lineY - 6, 470, panelHeight, HUD_PANEL_COLOR);
        DrawText("Pares con bounding boxes superpuestas:", 20, lineY, detailFontSize, HUD_TEXT_COLOR);
        lineY += 24;

        if (gameState.pairCollisions.empty())
        {
            DrawText("- ninguno", 20, lineY, detailFontSize, BOX_FREE_COLOR);
        }

        for (const PairCollision& pairCollision : gameState.pairCollisions)
        {
            const char* firstName = gameState.polyhedra[pairCollision.firstIndex].displayName.c_str();
            const char* secondName = gameState.polyhedra[pairCollision.secondIndex].displayName.c_str();

            if (pairCollision.result.areModelsColliding)
            {
                DrawText(TextFormat("- %s y %s: COLISIONAN", firstName, secondName), 20, lineY, detailFontSize, BOX_COLLIDING_COLOR);
            }
            else
            {
                DrawText(TextFormat("- %s y %s: solo cajas, no colisionan", firstName, secondName), 20, lineY, detailFontSize, BOX_OVERLAP_COLOR);
            }

            lineY += 22;
        }
    }

    static void DrawSelectedObjectInfo(const GameState& gameState)
    {
        const Polyhedron& selectedPolyhedron = gameState.polyhedra[gameState.selectedIndex];

        int panelWidth = 380;
        int panelX = GetScreenWidth() - panelWidth - 10;
        int lineY = 16;
        int fontSize = 18;
        int lineSpacing = 22;

        DrawRectangle(panelX, 10, panelWidth, 10 * lineSpacing + 12, HUD_PANEL_COLOR);
        int textX = panelX + 10;

        DrawText(TextFormat("Seleccionado: %s (%d/%d)", selectedPolyhedron.displayName.c_str(), gameState.selectedIndex + 1, static_cast<int>(gameState.polyhedra.size())),
                 textX, lineY, fontSize, { 255, 220, 40, 255 });
        lineY += lineSpacing;

        DrawText(TextFormat("Posicion: (%.2f, %.2f, %.2f)", selectedPolyhedron.position.x, selectedPolyhedron.position.y, selectedPolyhedron.position.z),
                 textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        DrawText(TextFormat("Rotacion: (%.0f, %.0f, %.0f)", selectedPolyhedron.rotationDegrees.x, selectedPolyhedron.rotationDegrees.y, selectedPolyhedron.rotationDegrees.z),
                 textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        DrawText(TextFormat("Escala: (%.2f, %.2f, %.2f)", selectedPolyhedron.scale.x, selectedPolyhedron.scale.y, selectedPolyhedron.scale.z),
                 textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        const AxisAlignedBox& boundingBox = selectedPolyhedron.boundingBox;
        DrawText(TextFormat("AABB min: (%.2f, %.2f, %.2f)", boundingBox.minCorner.x, boundingBox.minCorner.y, boundingBox.minCorner.z),
                 textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        DrawText(TextFormat("AABB max: (%.2f, %.2f, %.2f)", boundingBox.maxCorner.x, boundingBox.maxCorner.y, boundingBox.maxCorner.z),
                 textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        DrawText(TextFormat("Caras / planos: %d", static_cast<int>(selectedPolyhedron.facePlanes.size())), textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        bool normalsFacingCenter = AreAllNormalsFacingCenter(selectedPolyhedron);
        DrawText(TextFormat("Normales hacia el centro: %s", normalsFacingCenter ? "SI" : "NO"), textX, lineY, fontSize,
                 normalsFacingCenter ? BOX_FREE_COLOR : BOX_COLLIDING_COLOR);
        lineY += lineSpacing;

        const CollisionGrid& grid = gameState.collisionGrid;
        DrawText(TextFormat("Grilla: %d x %d x %d puntos", grid.pointsPerAxis, grid.pointsPerAxis, grid.pointsPerAxis), textX, lineY, fontSize, HUD_TEXT_COLOR);
        lineY += lineSpacing;

        DrawText(TextFormat("Separacion entre puntos: %.2f", grid.pointSpacing), textX, lineY, fontSize, HUD_TEXT_COLOR);
    }

    static void DrawControlsHelp()
    {
        const char* helpLines[] =
        {
            "TAB / SHIFT+TAB: cambiar objeto     Clic derecho + arrastrar: girar camara     Rueda: zoom",
            "Mover: A/D (X)  Q/E (Y)  W/S (Z)     Rotar: I/K (X)  J/L (Y)  U/O (Z)",
            "Escalar: R/F (mantener 1, 2 o 3 para un solo eje)     BACKSPACE: resetear objeto",
            "B: bounding boxes   N: normales   G: puntos de la grilla en ambos modelos   F1/F2: densidad de la grilla"
        };
        const int helpLineCount = 4;
        int fontSize = 16;
        int lineSpacing = 20;

        int startY = GetScreenHeight() - helpLineCount * lineSpacing - 12;
        DrawRectangle(10, startY - 6, GetScreenWidth() - 20, helpLineCount * lineSpacing + 12, HUD_PANEL_COLOR);

        for (int lineIndex = 0; lineIndex < helpLineCount; lineIndex++)
        {
            DrawText(helpLines[lineIndex], 20, startY + lineIndex * lineSpacing, fontSize, HUD_TEXT_COLOR);
        }
    }

    static void DrawMissingFilesScreen(const GameState& gameState)
    {
        DrawText("No se pudieron cargar estos modelos:", 40, 40, 28, BOX_COLLIDING_COLOR);

        int lineY = 90;

        for (const std::string& missingFile : gameState.missingFiles)
        {
            DrawText(TextFormat("- %s", missingFile.c_str()), 60, lineY, 22, HUD_TEXT_COLOR);
            lineY += 30;
        }

        DrawText("Tienen que estar en la carpeta res/models (al lado de src)", 40, lineY + 20, 22, HUD_TEXT_COLOR);
    }

    void RunGame()
    {
        SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Reyes - Colisiones con grilla de puntos");
        SetTargetFPS(60);

        GameState gameState;
        gameState.selectedIndex = 0;
        gameState.gridSpacingOption = DEFAULT_GRID_SPACING_OPTION;
        gameState.collisionGrid = CreateCollisionGrid({ 0.0f, 0.0f, 0.0f }, GRID_SIDE_LENGTH, GRID_SPACING_OPTIONS[DEFAULT_GRID_SPACING_OPTION]);
        gameState.showBoundingBoxes = true;
        gameState.showNormals = false;
        gameState.showSharedPoints = false;

        InitOrbitCamera(gameState.orbitCamera);
        LoadAllModels(gameState);

        bool areAllModelsLoaded = gameState.missingFiles.empty();

        while (!WindowShouldClose())
        {
            float deltaTime = GetFrameTime();

            if (areAllModelsLoaded)
            {
                HandleCameraInput(gameState.orbitCamera);
                HandleSelectedObjectInput(gameState, deltaTime);
                HandleOptionsInput(gameState);
                UpdateCollisions(gameState);
            }

            BeginDrawing();
            ClearBackground(BACKGROUND_COLOR);

            if (areAllModelsLoaded)
            {
                DrawScene(gameState);
                DrawCollisionStatus(gameState);
                DrawSelectedObjectInfo(gameState);
                DrawControlsHelp();
            }
            else
            {
                DrawMissingFilesScreen(gameState);
            }

            EndDrawing();
        }

        CloseWindow();
    }
}
