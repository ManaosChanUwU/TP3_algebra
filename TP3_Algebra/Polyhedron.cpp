
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

	// Actualiza los vertices del poliedro aplicando escala, rotacion y posicion.
	void UpdatePolyhedronWorldData(Polyhedron& polyhedron)
	{
		int vertexCount = static_cast<int>(polyhedron.localVertices.size());
		polyhedron.worldVertices.resize(vertexCount);

		// Convertimos los angulos de grados a radianes.
		float rotationRadiansX = polyhedron.rotationDegrees.x * DEGREES_TO_RADIANS;
		float rotationRadiansY = polyhedron.rotationDegrees.y * DEGREES_TO_RADIANS;
		float rotationRadiansZ = polyhedron.rotationDegrees.z * DEGREES_TO_RADIANS;

		// Vamos a sumar los vertices para calcular el centro del poliedro.
		Vector3 vertexSum = { 0.0f, 0.0f, 0.0f };

		// Transformamos cada vertice del modelo.
		for (int vertexIndex = 0; vertexIndex < vertexCount; vertexIndex++)
		{
			// 1. Aplicamos la escala.
			Vector3 transformedVertex = MultiplyComponents(polyhedron.localVertices[vertexIndex], polyhedron.scale);

			// 2. Aplicamos las rotaciones.
			transformedVertex = RotateAroundX(transformedVertex, rotationRadiansX);
			transformedVertex = RotateAroundY(transformedVertex, rotationRadiansY);
			transformedVertex = RotateAroundZ(transformedVertex, rotationRadiansZ);

			// 3. Aplicamos la posicion en el mundo.
			transformedVertex = AddVectors(transformedVertex, polyhedron.position);

			// Guardamos el vertice transformado.
			polyhedron.worldVertices[vertexIndex] = transformedVertex;

			// Acumulamos sus coordenadas para calcular el centro.
			vertexSum = AddVectors(vertexSum, transformedVertex);
		}

		// Si existen vertices, calculamos el promedio de sus posiciones.
		if (vertexCount > 0)
		{
			polyhedron.worldCenter = ScaleVector(vertexSum, 1.0f / static_cast<float>(vertexCount));
		}

		// Si no hay vertices, limpiamos los datos geometricos.
		if (vertexCount == 0)
		{
			polyhedron.boundingBox.minCorner = { 0.0f, 0.0f, 0.0f };
			polyhedron.boundingBox.maxCorner = { 0.0f, 0.0f, 0.0f };
			polyhedron.facePlanes.clear();
			return;
		}

		// ---------------------------------------------------------
		// BOUNDING BOX
		// ---------------------------------------------------------

		// Inicializamos los limites usando el primer vertice.
		// Esto evita asumir que el modelo esta centrado en el origen.
		polyhedron.boundingBox.minCorner = polyhedron.worldVertices[0];
		polyhedron.boundingBox.maxCorner = polyhedron.worldVertices[0];

		// Buscamos los valores minimos y maximos de cada eje.
		for (const Vector3& vertex : polyhedron.worldVertices)
		{
			// Limites del eje X.
			if (vertex.x < polyhedron.boundingBox.minCorner.x)
				polyhedron.boundingBox.minCorner.x = vertex.x;

			if (vertex.x > polyhedron.boundingBox.maxCorner.x)
				polyhedron.boundingBox.maxCorner.x = vertex.x;

			// Limites del eje Y.
			if (vertex.y < polyhedron.boundingBox.minCorner.y)
				polyhedron.boundingBox.minCorner.y = vertex.y;

			if (vertex.y > polyhedron.boundingBox.maxCorner.y)
				polyhedron.boundingBox.maxCorner.y = vertex.y;

			// Limites del eje Z.
			if (vertex.z < polyhedron.boundingBox.minCorner.z)
				polyhedron.boundingBox.minCorner.z = vertex.z;

			if (vertex.z > polyhedron.boundingBox.maxCorner.z)
				polyhedron.boundingBox.maxCorner.z = vertex.z;
		}

		// ---------------------------------------------------------
		// PLANOS DE LAS CARAS
		// ---------------------------------------------------------

		// Eliminamos los planos anteriores porque el poliedro pudo moverse,
		// rotarse o cambiar de escala.
		polyhedron.facePlanes.clear();

		// Creamos un plano por cada cara.
		for (const PolygonFace& face : polyhedron.faces)
		{
			// Necesitamos al menos tres vertices para construir un plano.
			if (face.vertexIndices.size() < 3)
				continue;

			// Obtenemos tres vertices de la cara en coordenadas del mundo.
			Vector3 pointA = polyhedron.worldVertices[face.vertexIndices[0]];
			Vector3 pointB = polyhedron.worldVertices[face.vertexIndices[1]];
			Vector3 pointC = polyhedron.worldVertices[face.vertexIndices[2]];

			// Construimos el plano usando esos tres puntos.
			FacePlane plane = CreatePlaneFromPoints(pointA, pointB, pointC);

			// Comprobamos hacia que lado apunta la normal.
			// Si el centro queda del lado negativo, la invertimos.
			float centerDistance = SignedDistanceToPlane(plane, polyhedron.worldCenter);

			if (centerDistance < 0.0f)
			{
				FlipPlane(plane);
			}

			// Guardamos el plano ya orientado hacia el centro.
			polyhedron.facePlanes.push_back(plane);
		}
	}

	// Comprueba si un punto esta dentro de un poliedro convexo.
	bool IsPointInsidePolyhedron(const Polyhedron& polyhedron, Vector3 point)
	{
		// Debe existir un plano por cada cara.
		if (polyhedron.facePlanes.size() != polyhedron.faces.size())
		{
			return false;
		}

		// Revisamos el punto respecto de todos los planos.
		for (const FacePlane& plane : polyhedron.facePlanes)
		{
			float distance = SignedDistanceToPlane(plane, point);

			// Como las normales apuntan hacia adentro,
			// el punto debe quedar del lado positivo de cada plano.
			// La tolerancia permite pequeños errores de redondeo.
			if (distance < -INSIDE_TOLERANCE)
			{
				return false;
			}
		}

		// Si supero todas las comprobaciones, esta dentro.
		return true;
	}

	// Comprueba que todas las normales de las caras apunten hacia el centro.
	bool AreAllNormalsFacingCenter(const Polyhedron& polyhedron)
	{
		// Si no hay un plano por cada cara, la geometria no esta completa.
		if (polyhedron.facePlanes.size() != polyhedron.faces.size())
		{
			return false;
		}

		// Comprobamos cada plano.
		for (const FacePlane& plane : polyhedron.facePlanes)
		{
			float centerDistance = SignedDistanceToPlane(plane, polyhedron.worldCenter);

			// El centro debe estar claramente del lado positivo.
			// Si esta en el lado negativo o sobre el plano, la normal
			// no esta apuntando correctamente hacia el interior.
			if (centerDistance <= INSIDE_TOLERANCE)
			{
				return false;
			}
		}

		return true;
	}

	// Calcula el centro geometrico de una cara.
	Vector3 GetFaceCenter(const Polyhedron& polyhedron, int faceIndex)
	{
		// Evitamos acceder a una cara inexistente.
		if (faceIndex < 0 || faceIndex >= static_cast<int>(polyhedron.faces.size()))
		{
			return { 0.0f, 0.0f, 0.0f };
		}

		const PolygonFace& face = polyhedron.faces[faceIndex];

		if (face.vertexIndices.empty())
		{
			return { 0.0f, 0.0f, 0.0f };
		}

		Vector3 vertexSum = { 0.0f, 0.0f, 0.0f };

		// Sumamos las posiciones de todos los vertices de la cara.
		for (int vertexIndex : face.vertexIndices)
		{
			vertexSum = AddVectors(vertexSum, polyhedron.worldVertices[vertexIndex]);
		}

		// Dividimos por la cantidad de vertices para obtener el promedio.
		return ScaleVector(vertexSum, 1.0f / static_cast<float>(face.vertexIndices.size()));
	}
}
