
#include "Geometry.h"

#include <cmath>
#include <algorithm>

namespace ReyesCuadra
{
    Vector3 AddVectors(Vector3 firstVector, Vector3 secondVector)
    {
        return { firstVector.x + secondVector.x, firstVector.y + secondVector.y, firstVector.z + secondVector.z };
    }

    Vector3 SubtractVectors(Vector3 firstVector, Vector3 secondVector)
    {
        return { firstVector.x - secondVector.x, firstVector.y - secondVector.y, firstVector.z - secondVector.z };
    }

    Vector3 ScaleVector(Vector3 vector, float scalar)
    {
        return { vector.x * scalar, vector.y * scalar, vector.z * scalar };
    }

    Vector3 MultiplyComponents(Vector3 firstVector, Vector3 secondVector)
    {
        return { firstVector.x * secondVector.x, firstVector.y * secondVector.y, firstVector.z * secondVector.z };
    }

    float DotProduct(Vector3 firstVector, Vector3 secondVector)
    {
        return firstVector.x * secondVector.x + firstVector.y * secondVector.y + firstVector.z * secondVector.z;
    }

    Vector3 CrossProduct(Vector3 firstVector, Vector3 secondVector)
    {
        return
        {
            firstVector.y * secondVector.z - firstVector.z * secondVector.y,
            firstVector.z * secondVector.x - firstVector.x * secondVector.z,
            firstVector.x * secondVector.y - firstVector.y * secondVector.x
        };
    }

    float VectorLength(Vector3 vector)
    {
        return std::sqrt(DotProduct(vector, vector));
    }

    Vector3 NormalizeVector(Vector3 vector)
    {
        float vectorLength = VectorLength(vector);

        if (vectorLength <= 0.0f)
        {
            return { 0.0f, 0.0f, 0.0f };
        }

        return ScaleVector(vector, 1.0f / vectorLength);
    }

    Vector3 RotateAroundX(Vector3 point, float angleRadians)
    {
        float cosine = std::cos(angleRadians);
        float sine = std::sin(angleRadians);

        return { point.x, point.y * cosine - point.z * sine, point.y * sine + point.z * cosine };
    }

    Vector3 RotateAroundY(Vector3 point, float angleRadians)
    {
        float cosine = std::cos(angleRadians);
        float sine = std::sin(angleRadians);

        return { point.x * cosine + point.z * sine, point.y, -point.x * sine + point.z * cosine };
    }

    Vector3 RotateAroundZ(Vector3 point, float angleRadians)
    {
        float cosine = std::cos(angleRadians);
        float sine = std::sin(angleRadians);

        return { point.x * cosine - point.y * sine, point.x * sine + point.y * cosine, point.z };
    }

    // Construye el plano que contiene los tres puntos recibidos.
    FacePlane CreatePlaneFromPoints(Vector3 pointA, Vector3 pointB, Vector3 pointC)
    {
        FacePlane plane;

        // Dos vectores que pertenecen al plano.
        Vector3 vectorAB = SubtractVectors(pointB, pointA);
        Vector3 vectorAC = SubtractVectors(pointC, pointA);

        // El producto vectorial genera un vector perpendicular al plano.
        // Lo normalizamos para que tenga longitud 1.
        plane.normal = NormalizeVector(CrossProduct(vectorAB, vectorAC));

        // Ecuación del plano:
        // normal . punto + distance = 0
        plane.distance = -DotProduct(plane.normal, pointA);

        return plane;
    }

    // Calcula la distancia con signo entre un punto y un plano.
    // Si da 0, el punto está sobre el plano.
    // Si da positivo o negativo, depende del lado hacia el que apunta la normal.
    float SignedDistanceToPlane(const FacePlane& plane, Vector3 point)
    {
        return DotProduct(plane.normal, point) + plane.distance;
    }

    // Invierte la orientación del plano.
    // Esto cambia el lado que se considera positivo y el negativo.
    void FlipPlane(FacePlane& plane)
    {
        plane.normal = ScaleVector(plane.normal, -1.0f);
        plane.distance *= -1.0f;
    }

    // Comprueba si dos cajas alineadas con los ejes se superponen.
    // Si solamente se tocan en un borde o una cara, también cuenta como solapamiento.
    bool AreBoxesOverlapping(const AxisAlignedBox& firstBox, const AxisAlignedBox& secondBox)
    {
        // Si una caja está completamente a la izquierda, derecha,
        // arriba, abajo, delante o detrás de la otra, no hay solapamiento.
        if (firstBox.maxCorner.x < secondBox.minCorner.x ||
            firstBox.minCorner.x > secondBox.maxCorner.x)
        {
            return false;
        }

        if (firstBox.maxCorner.y < secondBox.minCorner.y ||
            firstBox.minCorner.y > secondBox.maxCorner.y)
        {
            return false;
        }

        if (firstBox.maxCorner.z < secondBox.minCorner.z ||
            firstBox.minCorner.z > secondBox.maxCorner.z)
        {
            return false;
        }

        return true;
    }

    // Calcula la caja que representa la intersección de dos cajas.
    // Se supone que las cajas ya se comprobaron con AreBoxesOverlapping().
    AxisAlignedBox GetBoxIntersection(const AxisAlignedBox& firstBox, const AxisAlignedBox& secondBox)
    {
        AxisAlignedBox intersectionBox;

        // Para el mínimo elegimos el valor más grande de cada eje.
        intersectionBox.minCorner = {
            std::max(firstBox.minCorner.x, secondBox.minCorner.x),
            std::max(firstBox.minCorner.y, secondBox.minCorner.y),
            std::max(firstBox.minCorner.z, secondBox.minCorner.z)
        };

        // Para el máximo elegimos el valor más pequeño de cada eje.
        intersectionBox.maxCorner = {
            std::min(firstBox.maxCorner.x, secondBox.maxCorner.x),
            std::min(firstBox.maxCorner.y, secondBox.maxCorner.y),
            std::min(firstBox.maxCorner.z, secondBox.maxCorner.z)
        };

        return intersectionBox;
    }

    // Comprueba si un punto está dentro de una caja.
    // Se incluyen los bordes de la caja.
    bool IsPointInsideBox(const AxisAlignedBox& box, Vector3 point)
    {
        const float tolerance = 0.000001f;

        return
            point.x >= box.minCorner.x - tolerance &&
            point.x <= box.maxCorner.x + tolerance &&

            point.y >= box.minCorner.y - tolerance &&
            point.y <= box.maxCorner.y + tolerance &&

            point.z >= box.minCorner.z - tolerance &&
            point.z <= box.maxCorner.z + tolerance;
    }
}

