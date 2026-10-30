#include "Polyhedron.h"

#include <fstream>
#include <sstream>

namespace ReyesCuadra
{
    const float DEGREES_TO_RADIANS = 3.14159265f / 180.0f;
    const float INSIDE_TOLERANCE = 0.0001f;

    static int ParseVertexIndex(const std::string& faceToken, int loadedVertexCount)
    {
        size_t slashPosition = faceToken.find('/');
        std::string vertexPart = faceToken.substr(0, slashPosition);

        int objIndex = std::stoi(vertexPart);

        if (objIndex < 0)
        {
            return loadedVertexCount + objIndex;
        }

        return objIndex - 1;
    }

    bool LoadPolyhedronFromObj(const std::string& filePath, Polyhedron& polyhedron)
    {
        std::ifstream objFile(filePath);

        if (!objFile.is_open())
        {
            return false;
        }

        polyhedron.localVertices.clear();
        polyhedron.faces.clear();

        std::string fileLine;

        while (std::getline(objFile, fileLine))
        {
            std::istringstream lineStream(fileLine);
            std::string lineType;
            lineStream >> lineType;

            if (lineType == "v")
            {
                Vector3 vertexPosition = { 0.0f, 0.0f, 0.0f };
                lineStream >> vertexPosition.x >> vertexPosition.y >> vertexPosition.z;
                polyhedron.localVertices.push_back(vertexPosition);
            }
            else if (lineType == "f")
            {
                PolygonFace newFace;
                std::string faceToken;

                while (lineStream >> faceToken)
                {
                    int vertexIndex = ParseVertexIndex(faceToken, static_cast<int>(polyhedron.localVertices.size()));
                    newFace.vertexIndices.push_back(vertexIndex);
                }

                if (newFace.vertexIndices.size() >= 3)
                {
                    polyhedron.faces.push_back(newFace);
                }
            }
        }

        int vertexCount = static_cast<int>(polyhedron.localVertices.size());

        for (const PolygonFace& face : polyhedron.faces)
        {
            for (int vertexIndex : face.vertexIndices)
            {
                if (vertexIndex < 0 || vertexIndex >= vertexCount)
                {
                    return false;
                }
            }
        }

        polyhedron.position = { 0.0f, 0.0f, 0.0f };
        polyhedron.rotationDegrees = { 0.0f, 0.0f, 0.0f };
        polyhedron.scale = { 1.0f, 1.0f, 1.0f };
        polyhedron.isColliding = false;

        return vertexCount > 0 && !polyhedron.faces.empty();
    }

    // Pasa cada vertice del espacio local del modelo (como viene en el .obj) al
    // espacio del mundo, aplicando la escala, la rotacion y la posicion actuales.
    // Se llama en cada frame, asi que cualquier cambio que haga el jugador se ve
    // enseguida. Los vertices originales (localVertices) no se modifican nunca:
    // siempre se parte de ellos para no ir acumulando errores de redondeo.
    void UpdatePolyhedronWorldData(Polyhedron& polyhedron)
    {
        int vertexCount = static_cast<int>(polyhedron.localVertices.size());
        polyhedron.worldVertices.resize(vertexCount);

        // El jugador maneja la rotacion en grados, pero sin y cos trabajan en radianes
        float rotationRadiansX = polyhedron.rotationDegrees.x * DEGREES_TO_RADIANS;
        float rotationRadiansY = polyhedron.rotationDegrees.y * DEGREES_TO_RADIANS;
        float rotationRadiansZ = polyhedron.rotationDegrees.z * DEGREES_TO_RADIANS;

        // Acumula la suma de los vertices transformados para sacar el centro
        Vector3 vertexSum = { 0.0f, 0.0f, 0.0f };

        // Orden de la transformacion: escala -> rotacion (X, Y, Z) -> traslacion.
        // Escalar y rotar se hace con el modelo todavia alrededor de su origen,
        // asi gira y se estira sobre si mismo. Si primero se lo moviera, rotaria
        // alrededor del origen del mundo (como un planeta orbitando) y la escala
        // tambien lo alejaria o acercaria al origen.
        for (int vertexIndex = 0; vertexIndex < vertexCount; vertexIndex++)
        {
            // 1) Escala: cada coordenada se multiplica por la escala de su eje
            Vector3 transformedVertex = MultiplyComponents(polyhedron.localVertices[vertexIndex], polyhedron.scale);

            // 2) Rotacion: se gira alrededor de cada eje del mundo, uno despues del otro
            transformedVertex = RotateAroundX(transformedVertex, rotationRadiansX);
            transformedVertex = RotateAroundY(transformedVertex, rotationRadiansY);
            transformedVertex = RotateAroundZ(transformedVertex, rotationRadiansZ);

            // 3) Traslacion: se suma la posicion del modelo en el mundo
            transformedVertex = AddVectors(transformedVertex, polyhedron.position);

            polyhedron.worldVertices[vertexIndex] = transformedVertex;
            vertexSum = AddVectors(vertexSum, transformedVertex);
        }

        // Centro del modelo: promedio de los vertices transformados.
        // En un poliedro convexo este punto siempre queda adentro, y despues se
        // va a usar como referencia para orientar las normales hacia el centro.
        if (vertexCount > 0)
        {
            polyhedron.worldCenter = ScaleVector(vertexSum, 1.0f / static_cast<float>(vertexCount));
        }

        // Pendiente (pasos siguientes): bounding box y planos de las caras
        polyhedron.boundingBox.minCorner = { 0.0f, 0.0f, 0.0f };
        polyhedron.boundingBox.maxCorner = { 0.0f, 0.0f, 0.0f };
        polyhedron.facePlanes.clear();
    }

    bool IsPointInsidePolyhedron(const Polyhedron& polyhedron, Vector3 point)
    {
        return false;
    }

    bool AreAllNormalsFacingCenter(const Polyhedron& polyhedron)
    {
        return false;
    }

    Vector3 GetFaceCenter(const Polyhedron& polyhedron, int faceIndex)
    {
        const PolygonFace& face = polyhedron.faces[faceIndex];
        Vector3 vertexSum = { 0.0f, 0.0f, 0.0f };

        for (int vertexIndex : face.vertexIndices)
        {
            vertexSum = AddVectors(vertexSum, polyhedron.worldVertices[vertexIndex]);
        }

        return ScaleVector(vertexSum, 1.0f / static_cast<float>(face.vertexIndices.size()));
    }
}
