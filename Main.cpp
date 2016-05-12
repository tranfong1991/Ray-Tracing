#include<fstream>
#include<math.h>
#include"HelperClasses.h"
#include<GL/glut.h>

/******************************************************************
Notes:
This is the same utility as in the earlier homework assignment.
Image size is 400 by 400 by default.  You may adjust this if
you want to.
You can assume the window will NOT be resized.
Call clearFramebuffer to clear the entire framebuffer.
Call setFramebuffer to set a pixel.  This should be the only
routine you use to set the color (other than clearing the
entire framebuffer).  drawit() will cause the current
framebuffer to be displayed.
As is, your ray tracer should probably be called from
within the display function.  There is a very short sample
of code there now.  You can replace that with a call to a
function that raytraces your scene (or whatever other
interaction you provide.
You may add code to any of the subroutines here,  You probably
want to leave the drawit, clearFramebuffer, and
setFramebuffer commands alone, though.
*****************************************************************/

#define ImageW 400
#define ImageH 400
#define MAX_DEPTH 10		//maximum depth for the ray tracing tree

float framebuffer[ImageH][ImageW][3];
Color ambientIntensity(0.5, 0.5, 0.5);

float max(float n1, float n2)
{
	return n1 > n2 ? n1 : n2;
}

// Draws the scene
void drawit(void)
{
	glDrawPixels(ImageW, ImageH, GL_RGB, GL_FLOAT, framebuffer);
	glFlush();
}

// Clears framebuffer to black
void clearFramebuffer()
{
	int i, j;

	for (i = 0; i < ImageH; i++) {
		for (j = 0; j < ImageW; j++) {
			framebuffer[i][j][0] = 0.0;
			framebuffer[i][j][1] = 0.0;
			framebuffer[i][j][2] = 0.0;
		}
	}
}

void setFramebuffer(int x, int y, float R, float G, float B)
{
	// changes the origin from the lower-left corner to the upper-left corner
	y = ImageH - 1 - y;
	if (R <= 1.0)
		if (R >= 0.0)
			framebuffer[y][x][0] = R;
		else
			framebuffer[y][x][0] = 0.0;
	else
		framebuffer[y][x][0] = 1.0;
	if (G <= 1.0)
		if (G >= 0.0)
			framebuffer[y][x][1] = G;
		else
			framebuffer[y][x][1] = 0.0;
	else
		framebuffer[y][x][1] = 1.0;
	if (B <= 1.0)
		if (B >= 0.0)
			framebuffer[y][x][2] = B;
		else
			framebuffer[y][x][2] = 0.0;
	else
		framebuffer[y][x][2] = 1.0;
}

float magnitude(Vector& v)
{
	return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

// Normalizes the vector passed in
void normalize(Vector& v)
{
	float temp = magnitude(v);

	if (temp > 0.0) {
		v.x /= temp;
		v.y /= temp;
		v.z /= temp;
	}
	else {
		v.x = 0.0;
		v.y = 0.0;
		v.z = 0.0;
	}
}

// Returns dot product of two vectors
float dot(Vector& v1, Vector& v2)
{
	return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}

void lineSphereIntersect(Intersection& intersection, Point& point, Vector& line, Sphere& sphere)
{
	const Point& sphereCenter = sphere.getCenter();

	//from eye position to center of sphere
	Vector d(point.x - sphereCenter.x, point.y - sphereCenter.y, point.z - sphereCenter.z);

	float a = 1;
	float b = 2 * dot(line, d);
	float c = pow(magnitude(d), 2) - pow(sphere.getRadius(), 2);
	float delta = pow(b, 2) - 4 * a * c;

	if (delta < 0){
		intersection.numOfPoints = 0;
	}
	else if (delta == 0){
		intersection.numOfPoints = 1;
		intersection.t = new float[1];
		intersection.t[0] = -b / (2 * a);
	}
	else {
		intersection.numOfPoints = 2;
		intersection.t = new float[2];
		intersection.t[0] = (-b - sqrt(delta)) / (2 * a);
		intersection.t[1] = (-b + sqrt(delta)) / (2 * a);
	}
}

void linePlaneIntersect(Intersection& intersection, Point& point, Vector& line, Plane& plane)
{
	const Point& pointOnPlane = plane.getPoint();

	Vector planeNormal = plane.getNormal();
	Vector d(pointOnPlane.x - point.x, pointOnPlane.y - point.y, pointOnPlane.z - point.z);
	float denom = dot(line, planeNormal);

	if (denom != 0){
		intersection.numOfPoints = 1;
		intersection.t = new float[1];
		intersection.t[0] = dot(d, planeNormal) / denom;
	}
	else {
		intersection.numOfPoints = 0;
	}
}

Vector sphereNormalAt(Point& point, Sphere& sphere)
{
	const Point& center = sphere.getCenter();
	Vector normal(point.x - center.x, point.y - center.y, point.z - center.z);
	normalize(normal);

	return normal;
}

Vector reflectedVectorAt(Vector& incident, Vector& surfaceNormal)
{
	float dotLightSurfaceNormal = dot(incident, surfaceNormal);
	Vector& reflectedVector = 2 * dotLightSurfaceNormal * surfaceNormal - incident;
	normalize(reflectedVector);

	return reflectedVector;
}

Color recursiveReflection(int depth, Vector& ray, Vector& surfaceNormal, Point& point, Sphere currentSphere, Sphere* spheres, int numSpheres, Light* lightSources, int numLights, Plane* infinitePlanes, int numPlanes)
{
	Vector& flippedRay = (-1)*ray;
	Vector& reflected = reflectedVectorAt(flippedRay, surfaceNormal);

	for (int s = 0; s < numSpheres; s++){
		if (spheres[s] == currentSphere)
			continue;

		Sphere& sphere = spheres[s];

		Intersection si;
		lineSphereIntersect(si, point, reflected, sphere);

		if (si.numOfPoints > 0 && (si.t[0] > 0 || si.t[1] > 0)){
			Point intersectPoint;

			if (si.numOfPoints == 1){
				intersectPoint = Point(point.x + si.t[0] * reflected.x, point.y + si.t[0] * reflected.y, point.z + si.t[0] * reflected.z);
			}
			else {
				float clostT = si.t[0] < si.t[1] ? si.t[0] : si.t[1];
				intersectPoint = Point(point.x + clostT * reflected.x, point.y + clostT * reflected.y, point.z + clostT * reflected.z);
			}

			Vector& normal = sphereNormalAt(intersectPoint, sphere);
			Vector eyeVector(point.x - intersectPoint.x, point.y - intersectPoint.y, point.z - intersectPoint.z);
			normalize(eyeVector);	

			const MaterialProperties& sphereProperties = sphere.getProperties();
			Color& sphereAmbient = sphereProperties.ambientCoeff * ambientIntensity;
			Color& directIlum = sphereAmbient;

			for (int l = 0; l < numLights; l++){
				const Point& lightPosition = lightSources[l].getPosition();
				const Color& lightIntensity = lightSources[l].getIntensity();

				Vector lightVector(lightPosition.x - intersectPoint.x, lightPosition.y - intersectPoint.y, lightPosition.z - intersectPoint.z);
				normalize(lightVector);

				//draw shadow
				//if that point is occluded by its own sphere
				if (dot(normal, lightVector) <= 0)
					continue;

				//otherwise, check if the light is occluded by other spheres
				bool hasIntersect = false;
				for (int ss = 0; ss < numSpheres; ss++){
					if (spheres[ss] == sphere)
						continue;

					Intersection ssi;
					lineSphereIntersect(ssi, intersectPoint, lightVector, spheres[ss]);

					//check if the intersection point is between the object and the light	
					if (ssi.numOfPoints > 0 && (ssi.t[0] > 0 || ssi.t[1] > 0)){
						hasIntersect = true;
						break;
					}
				}
				if (hasIntersect)
					continue;

				float dotLightSurfaceNormal = dot(lightVector, normal);
				Color& sphereDiffuse = dotLightSurfaceNormal * (sphereProperties.diffuseCoeff * lightIntensity);

				Vector& reflectedVector = reflectedVectorAt(lightVector, normal);
				Color& sphereSpecular = pow(max(dot(reflectedVector, eyeVector), 0), sphereProperties.specularExpo) * (sphereProperties.specularCoeff * lightIntensity);

				directIlum = directIlum + sphereDiffuse + sphereSpecular;
			}

			if (depth == MAX_DEPTH)
				return directIlum;
			return directIlum + sphereProperties.reflectance * recursiveReflection(depth + 1, reflected, 
				normal, intersectPoint, sphere, spheres, numSpheres, lightSources, numLights, infinitePlanes, numPlanes);
		}
	}

	for (int p = 1; p > 0; p--){
		Vector planeNormal = infinitePlanes[p].getNormal();
		const MaterialProperties& planeProperties = infinitePlanes[p].getProperties();
		Color& planeAmbient = planeProperties.ambientCoeff * ambientIntensity;

		Intersection planeIntersection;
		linePlaneIntersect(planeIntersection, point, reflected, infinitePlanes[p]);

		if (planeIntersection.numOfPoints > 0 && planeIntersection.t[0] > 0) {
			Color& directIlum = planeAmbient;
			float* t = planeIntersection.t;
			Point intersectPoint(point.x + t[0] * reflected.x, point.y + t[0] * reflected.y, point.z + t[0] * reflected.z);

			for (int l = 0; l < numLights; l++){
				const Point& lightPosition = lightSources[l].getPosition();
				const Color& lightIntensity = lightSources[l].getIntensity();

				Vector lightVector(lightPosition.x - intersectPoint.x, lightPosition.y - intersectPoint.y, lightPosition.z - intersectPoint.z);
				normalize(lightVector);

				//draw shadow
				bool hasIntersect = false;
				for (int ss = 0; ss < numSpheres; ss++){
					Intersection sphereIntersection;
					lineSphereIntersect(sphereIntersection, intersectPoint, lightVector, spheres[ss]);

					if (sphereIntersection.numOfPoints > 0){
						hasIntersect = true;
						break;
					}
				}
				if (hasIntersect)
					continue;

				float dotLightSurfaceNormal = dot(lightVector, planeNormal);
				Color& planeDiffuse = dotLightSurfaceNormal * (planeProperties.diffuseCoeff * lightIntensity);

				directIlum = directIlum + planeDiffuse;
			}
			return directIlum;
		}
	}
	return Color(0, 0, 0);	//no intersection
}


//work on shadow and reflection. FIX eyePosition, should we have SHADOW OVERLAPPING?
void rayTrace(Camera& camera, Plane& viewPort, Light* lightSources, int numLights, Sphere* spheres, int numSpheres, Plane* infinitePlanes, int numPlanes)
{
	const Point& eyePosition = camera.getEye();
	const Point& viewPortPosition = viewPort.getPoint();

	for (int y = 0; y < ImageH; y++){
		for (int x = 0; x < ImageW; x++){
			Vector ray((x - eyePosition.x) / 200.0, (y - eyePosition.y) / 200.0, viewPortPosition.z - eyePosition.z);
			normalize(ray);

			//draw plane and shadow
			for (int p = 0; p < numPlanes; p++){
				Vector planeNormal = infinitePlanes[p].getNormal();
				const MaterialProperties& planeProperties = infinitePlanes[p].getProperties();
				Color& planeAmbient = planeProperties.ambientCoeff * ambientIntensity;

				Intersection planeIntersection;
				linePlaneIntersect(planeIntersection, Point(camera.getEye()), ray, infinitePlanes[p]);

				if (planeIntersection.numOfPoints != 0 && planeIntersection.t[0] > 0) {
					Color& directIlum = planeAmbient;
					float* t = planeIntersection.t;
					Point intersectPoint(eyePosition.x + t[0] * ray.x, eyePosition.y + t[0] * ray.y, eyePosition.z + t[0] * ray.z);
						
					for (int l = 0; l < numLights; l++){
						const Point& lightPosition = lightSources[l].getPosition();
						const Color& lightIntensity = lightSources[l].getIntensity();

						Vector lightVector(lightPosition.x - intersectPoint.x, lightPosition.y - intersectPoint.y, lightPosition.z - intersectPoint.z);
						normalize(lightVector);

						//draw shadow
						bool hasIntersect = false;
						for (int s = 0; s < numSpheres; s++){
							Intersection sphereIntersection;
							lineSphereIntersect(sphereIntersection, intersectPoint, lightVector, spheres[s]);

							if (sphereIntersection.numOfPoints > 0){
								hasIntersect = true;
								break;
							}
						}
						if (hasIntersect)
							continue;

						float dotLightSurfaceNormal = dot(lightVector, planeNormal);
						Color& planeDiffuse = dotLightSurfaceNormal * (planeProperties.diffuseCoeff * lightIntensity);

						directIlum = directIlum + planeDiffuse;
					}
					setFramebuffer(x, y, directIlum.r, directIlum.g, directIlum.b);
				}
			}

			//draw spheres and shadows
			for (int s = 0; s < numSpheres; s++){
				Sphere& sphere = spheres[s];
				const MaterialProperties& sphereProperties = sphere.getProperties();
				Color& sphereAmbient = sphereProperties.ambientCoeff * ambientIntensity;

				Intersection sphereIntersection;
				lineSphereIntersect(sphereIntersection, Point(camera.getEye()), ray, sphere);

				if (sphereIntersection.numOfPoints != 0){
					float* t = sphereIntersection.t;
					Point intersectPoint;

					if (sphereIntersection.numOfPoints == 1){
						intersectPoint = Point(eyePosition.x + t[0] * ray.x, eyePosition.y + t[0] * ray.y, eyePosition.z + t[0] * ray.z);
					}
					else {
						float closestT = t[0] < t[1] ? t[0] : t[1];
						intersectPoint = Point(eyePosition.x + closestT * ray.x, eyePosition.y + closestT * ray.y, eyePosition.z + closestT * ray.z);;
					}

					Vector eyeVector(eyePosition.x - intersectPoint.x, eyePosition.y - intersectPoint.y, eyePosition.z - intersectPoint.z);
					normalize(eyeVector);
					Vector& surfaceNormal = sphereNormalAt(intersectPoint, sphere);
					Color& directIlum = sphereAmbient;

					for (int l = 0; l < numLights; l++){
						const Point& lightPosition = lightSources[l].getPosition();
						const Color& lightIntensity = lightSources[l].getIntensity();

						Vector lightVector(lightPosition.x - intersectPoint.x, lightPosition.y - intersectPoint.y, lightPosition.z - intersectPoint.z);
						normalize(lightVector);

						//draw shadow
						//if that point is occluded by its own sphere
						if (dot(surfaceNormal, lightVector) <= 0)
							continue;

						//otherwise, check if the light is occluded by other spheres
						bool hasIntersect = false;
						for (int ss = 0; ss < numSpheres; ss++){
							if (spheres[ss] == sphere)
								continue;

							Intersection ssi;
							lineSphereIntersect(ssi, intersectPoint, lightVector, spheres[ss]);

							//check if the intersection point is between the object and the light	
							if (ssi.numOfPoints > 0 && (ssi.t[0] > 0 || ssi.t[1] > 0)){
								hasIntersect = true;
								break;
							}
						}
						if (hasIntersect)
							continue;

						float dotLightSurfaceNormal = dot(lightVector, surfaceNormal);
						Color& sphereDiffuse = dotLightSurfaceNormal * (sphereProperties.diffuseCoeff * lightIntensity);

						Vector& reflectedVector = reflectedVectorAt(lightVector, surfaceNormal);
						Color& sphereSpecular = pow(max(dot(reflectedVector, eyeVector), 0), sphereProperties.specularExpo) * (sphereProperties.specularCoeff * lightIntensity);

						directIlum = directIlum + sphereDiffuse + sphereSpecular;
					} 
					directIlum = directIlum + sphereProperties.reflectance *
						recursiveReflection(1, ray, surfaceNormal, intersectPoint, sphere, spheres, numSpheres, lightSources, numLights, infinitePlanes, numPlanes);
					setFramebuffer(x, y, directIlum.r, directIlum.g, directIlum.b);
				}
			}
		}
	}
}

void display(void)
{
	Plane viewPort(Point(0, 0, 5), Vector(0, 0, -1));
	Camera camera(Point(200, 201, 0), Point(200, 200, 5), Vector(0, -1, 0));

	//2 light sources
	Light l1(Point(100, 100, 50), Color(1.0, 1.0, 1.0));
	Light l2(Point(300, 150, 100), Color(1.0, 1.0, 0.0));

	//4 spheres
	Sphere s1(Point(198.9, 200.6, 15), 1.4);		//purple sphere
	MaterialProperties smp1;
	smp1.ambientCoeff = Coefficients(0.1, 0.0, 0.2);
	smp1.diffuseCoeff = Coefficients(0.7, 0.4, 1.0);
	smp1.specularCoeff = Coefficients(1.0, 1.0, 1.0);
	smp1.specularExpo = 100.0;
	smp1.reflectance = 1.0;
	s1.setProperties(smp1);

	Sphere s2(Point(199.5, 201.5, 10), 0.5);		//red sphere
	MaterialProperties smp2;
	smp2.ambientCoeff = Coefficients(0.4, 0.0, 0.0);
	smp2.diffuseCoeff = Coefficients(1.0, 0.0, 0.0);
	smp2.specularCoeff = Coefficients(1.0, 1.0, 1.0);
	smp2.specularExpo = 5.0;
	smp2.reflectance = 0.0;
	s2.setProperties(smp2);

	Sphere s3(Point(201.5, 200.8, 15), 1.2);		//gray sphere
	MaterialProperties smp3;
	smp3.ambientCoeff = Coefficients(0.5, 0.5, 0.5);
	smp3.diffuseCoeff = Coefficients(0.8, 0.8, 0.8);
	smp3.specularCoeff = Coefficients(0.9, 0.9, 0.9);
	smp3.specularExpo = 50.0;
	smp3.reflectance = 0.6;
	s3.setProperties(smp3);

	Sphere s4(Point(200.8, 201.55, 10.5), 0.45);	//green sphere
	MaterialProperties smp4;
	smp4.ambientCoeff = Coefficients(0.098, 0.2, 0.0);
	smp4.diffuseCoeff = Coefficients(0.6, 1.0, 0.2);
	smp4.specularCoeff = Coefficients(1.0, 1.0, 1.0);
	smp4.specularExpo = 80.0;
	smp4.reflectance = 0.3;
	s4.setProperties(smp4);

	//2 infinite planes
	Plane p1(Point(200, 200, 50), Vector(0, 0, -1));	//vertical plane
	MaterialProperties pmp1;
	pmp1.ambientCoeff = Coefficients(0.4, 0.7, 1.0);
	pmp1.diffuseCoeff = Coefficients(0.4, 1.0, 1.0);
	pmp1.reflectance = 0.0;
	p1.setProperties(pmp1);

	Plane p2(Point(200, 202, 25), Vector(0, -1, 0));	//horizontal plane
	MaterialProperties pmp2;
	pmp2.ambientCoeff = Coefficients(0.12, 0.12, 0.12);
	pmp2.diffuseCoeff = Coefficients(0.37, 0.37, 0.37);
	pmp2.reflectance = 0.9;
	p2.setProperties(pmp2);

	Light lightSources[] = { l1, l2 };
	Sphere spheres[] = { s1, s2, s3, s4};
	Plane infinitePlanes[] = { p1, p2 };

	rayTrace(camera, viewPort, lightSources, 2, spheres, 4, infinitePlanes, 2);
	drawit();
}

void init(void)
{
	drawit();
	clearFramebuffer();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(ImageW, ImageH);
	glutInitWindowPosition(700, 300);
	glutCreateWindow("Phong Tran - Homework 5");

	init();
	glutDisplayFunc(display);
	glutMainLoop();

	return 0;
}