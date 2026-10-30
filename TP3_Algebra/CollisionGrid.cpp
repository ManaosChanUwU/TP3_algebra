
#include "CollisionGrid.h"

#include <cmath>
#include <algorithm>

namespace ReyesCuadra
{
    // Crea una grilla tridimensional de puntos equidistantes.
    // centerPoint indica el centro del cubo y sideLength su tamaño.
    CollisionGrid CreateCollisionGrid(Vector3 centerPoint, float sideLength, float pointSpacing)
    {
        CollisionGrid grid;

        // Evitamos valores invalidos.
        if (sideLength <= 0.0f || pointSpacing <= 0.0f)
        {
            grid.minCorner = centerPoint;
            grid.pointSpacing = 0.0f;
            grid.pointsPerAxis = 0;
            return grid;
        }

        // Calculamos la esquina minima del cubo.
        // Restamos la mitad del tamaño al centro en cada eje.
        float halfSideLength = sideLength * 0.5f;

        grid.minCorner = {
            centerPoint.x - halfSideLength,
            centerPoint.y - halfSideLength,
            centerPoint.z - halfSideLength
        };

        grid.pointSpacing = pointSpacing;

        // Calculamos cuantos puntos entran en cada eje.
        // Sumamos 1 porque tambien contamos el punto inicial.
        grid.pointsPerAxis = static_cast<int>(std::floor(sideLength / pointSpacing + 0.00001f)) + 1;

        return grid;
    }

    // Obtiene la posicion de un punto de la grilla usando sus indices.
    // indexX, indexY e indexZ indican su ubicacion en cada eje.
    Vector3 GetGridPoint(const CollisionGrid& grid, int indexX, int indexY, int indexZ)
    {
        // Comprobamos que la grilla y los indices sean validos.
        if (grid.pointsPerAxis <= 0 || grid.pointSpacing <= 0.0f ||
            indexX < 0 || indexX >= grid.pointsPerAxis ||
            indexY < 0 || indexY >= grid.pointsPerAxis ||
            indexZ < 0 || indexZ >= grid.pointsPerAxis)
        {
            return { 0.0f, 0.0f, 0.0f };
        }

        // La posicion se calcula sumando el desplazamiento
        // correspondiente a cada indice desde la esquina minima.
        return {
            grid.minCorner.x + indexX * grid.pointSpacing,
            grid.minCorner.y + indexY * grid.pointSpacing,
            grid.minCorner.z + indexZ * grid.pointSpacing
        };
    }

    // Devuelve la caja que contiene todos los puntos de la grilla.
    AxisAlignedBox GetGridBounds(const CollisionGrid& grid)
    {
        AxisAlignedBox gridBounds;

        gridBounds.minCorner = grid.minCorner;

        // Si la grilla no es valida, devolvemos una caja sin tamaño.
        if (grid.pointsPerAxis <= 0 || grid.pointSpacing <= 0.0f)
        {
            gridBounds.maxCorner = grid.minCorner;
            return gridBounds;
        }

        // Calculamos la posicion del ultimo punto de cada eje.
        float totalSpacing = (grid.pointsPerAxis - 1) * grid.pointSpacing;

        gridBounds.maxCorner = {
            grid.minCorner.x + totalSpacing,
            grid.minCorner.y + totalSpacing,
            grid.minCorner.z + totalSpacing
        };

        return gridBounds;
    }

    // Calcula los indices inicial y final de los puntos de la grilla
    // que pueden encontrarse dentro de una caja.
    //
    // Esto permite recorrer solamente la zona de interes en lugar
    // de revisar todos los puntos de la grilla.
    static bool GetGridIndexRange(
        const CollisionGrid& grid,
        const AxisAlignedBox& box,
        int& minX, int& maxX,
        int& minY, int& maxY,
        int& minZ, int& maxZ)
    {
        if (grid.pointsPerAxis <= 0 || grid.pointSpacing <= 0.0f)
        {
            return false;
        }

        AxisAlignedBox gridBounds = GetGridBounds(grid);

        // Si la caja no se superpone con los limites de la grilla,
        // no hay puntos que podamos revisar.
        if (!AreBoxesOverlapping(box, gridBounds))
        {
            return false;
        }

        // Convertimos las coordenadas del mundo a indices de grilla.
        // ceil encuentra el primer indice que puede entrar en la caja.
        // floor encuentra el ultimo indice que puede entrar en la caja.
        minX = static_cast<int>(std::ceil(
            (box.minCorner.x - grid.minCorner.x) / grid.pointSpacing - 0.00001f
        ));

        maxX = static_cast<int>(std::floor(
            (box.maxCorner.x - grid.minCorner.x) / grid.pointSpacing + 0.00001f
        ));

        minY = static_cast<int>(std::ceil(
            (box.minCorner.y - grid.minCorner.y) / grid.pointSpacing - 0.00001f
        ));

        maxY = static_cast<int>(std::floor(
            (box.maxCorner.y - grid.minCorner.y) / grid.pointSpacing + 0.00001f
        ));

        minZ = static_cast<int>(std::ceil(
            (box.minCorner.z - grid.minCorner.z) / grid.pointSpacing - 0.00001f
        ));

        maxZ = static_cast<int>(std::floor(
            (box.maxCorner.z - grid.minCorner.z) / grid.pointSpacing + 0.00001f
        ));

        // Limitamos los indices para que no salgan de la grilla.
        minX = std::max(0, minX);
        minY = std::max(0, minY);
        minZ = std::max(0, minZ);

        maxX = std::min(grid.pointsPerAxis - 1, maxX);
        maxY = std::min(grid.pointsPerAxis - 1, maxY);
        maxZ = std::min(grid.pointsPerAxis - 1, maxZ);

        // Si los rangos no tienen puntos validos, terminamos.
        return minX <= maxX && minY <= maxY && minZ <= maxZ;
    }

    // Comprueba si dos poliedros colisionan utilizando la grilla.
    CollisionResult CheckPolyhedronCollision(
        const Polyhedron& firstPolyhedron,
        const Polyhedron& secondPolyhedron,
        const CollisionGrid& grid)
    {
        CollisionResult collisionResult;

        collisionResult.areBoxesOverlapping = false;
        collisionResult.areModelsColliding = false;
        collisionResult.sharedGridPoint = { 0.0f, 0.0f, 0.0f };

        // PRIMERA ETAPA:
        // Comprobamos si las Bounding Boxes se superponen.
        // Si no lo hacen, no tiene sentido revisar los puntos.
        if (!AreBoxesOverlapping(firstPolyhedron.boundingBox, secondPolyhedron.boundingBox))
        {
            return collisionResult;
        }

        collisionResult.areBoxesOverlapping = true;

        // SEGUNDA ETAPA:
        // Calculamos la interseccion entre ambas Bounding Boxes.
        AxisAlignedBox intersectionBox = GetBoxIntersection(
            firstPolyhedron.boundingBox,
            secondPolyhedron.boundingBox
        );

        // Obtenemos los indices de los puntos de la grilla
        // que estan dentro de la zona de interseccion.
        int minX, maxX;
        int minY, maxY;
        int minZ, maxZ;

        if (!GetGridIndexRange(
            grid,
            intersectionBox,
            minX, maxX,
            minY, maxY,
            minZ, maxZ))
        {
            return collisionResult;
        }

        // Recorremos los puntos de la grilla dentro de la interseccion.
        for (int x = minX; x <= maxX; x++)
        {
            for (int y = minY; y <= maxY; y++)
            {
                for (int z = minZ; z <= maxZ; z++)
                {
                    Vector3 currentPoint = GetGridPoint(grid, x, y, z);

                    // El punto tiene que estar dentro de los dos poliedros.
                    bool insideFirst = IsPointInsidePolyhedron(firstPolyhedron, currentPoint);

                    if (!insideFirst)
                    {
                        continue;
                    }

                    bool insideSecond = IsPointInsidePolyhedron(secondPolyhedron, currentPoint);

                    if (!insideSecond)
                    {
                        continue;
                    }

                    // Encontramos un punto compartido.
                    // Ya podemos afirmar que existe colision segun la grilla.
                    collisionResult.areModelsColliding = true;
                    collisionResult.sharedGridPoint = currentPoint;

                    return collisionResult;
                }
            }
        }

        // Las cajas se superponen, pero ningun punto de la grilla
        // fue encontrado dentro de ambos poliedros.
        return collisionResult;
    }

    // Recopila los puntos de la grilla que estan dentro de ambos poliedros.
    // Se utiliza para dibujar visualmente los puntos compartidos.
    void CollectSharedGridPoints(
        const Polyhedron& firstPolyhedron,
        const Polyhedron& secondPolyhedron,
        const CollisionGrid& grid,
        std::vector<Vector3>& sharedPoints,
        int maxPointCount)
    {
        // Si no hay espacio para guardar puntos, terminamos.
        if (maxPointCount <= 0 || static_cast<int>(sharedPoints.size()) >= maxPointCount)
        {
            return;
        }

        // Comprobamos primero si las cajas se superponen.
        if (!AreBoxesOverlapping(firstPolyhedron.boundingBox, secondPolyhedron.boundingBox))
        {
            return;
        }

        // Calculamos la zona compartida entre las cajas.
        AxisAlignedBox intersectionBox = GetBoxIntersection(
            firstPolyhedron.boundingBox,
            secondPolyhedron.boundingBox
        );

        int minX, maxX;
        int minY, maxY;
        int minZ, maxZ;

        // Obtenemos los indices de la grilla que caen dentro de esa zona.
        if (!GetGridIndexRange(
            grid,
            intersectionBox,
            minX, maxX,
            minY, maxY,
            minZ, maxZ))
        {
            return;
        }

        // Recorremos solamente los puntos de la zona compartida.
        for (int x = minX; x <= maxX; x++)
        {
            for (int y = minY; y <= maxY; y++)
            {
                for (int z = minZ; z <= maxZ; z++)
                {
                    // Respetamos el limite maximo de puntos a dibujar.
                    if (static_cast<int>(sharedPoints.size()) >= maxPointCount)
                    {
                        return;
                    }

                    Vector3 currentPoint = GetGridPoint(grid, x, y, z);

                    // Comprobamos si el punto esta dentro de ambos poliedros.
                    if (IsPointInsidePolyhedron(firstPolyhedron, currentPoint) &&
                        IsPointInsidePolyhedron(secondPolyhedron, currentPoint))
                    {
                        sharedPoints.push_back(currentPoint);
                    }
                }
            }
        }
    }
}

