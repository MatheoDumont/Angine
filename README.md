# Angine
Angine is an implementation of a Physics Engine in Rust with basic physics simulation and collision detection.
It uses a discrete collision engine, solve collisions via impulsions and compute each step of the collision using the Semi-Implicit Euler method.

It's currently supported shapes are Sphere, Plane and OBB (Oriented Bounding Box) .

Intersection:
|        | Sphere | Plane | OBB |
|--------|--------|-------|-----|
| Sphere |   x    |   x   |  x  |
| Plane  |        |   x   |  x  |
| OBB    |        |       |  x  |

  
Collision Contact creation:
|        | Sphere | Plane | OBB |
|--------|--------|-------|-----|
| Sphere |   x    |       |     |
| Plane  |        |       |  x  |
| OBB    |        |       |  x  |


There is currently no broadphase, collision detection iterate over all the objects to detect a collision.
For each collision detected, a contact manifold is created. It contain the normal of the collision, the points on which the collision occur and pointers to the objects.

Then, each collision is resolved by computing an impulsion used to push objects apart from each other.  
Finally, we compute the new state of each object in the simulation by integrating the velocities then the position and orientation.

Here is the demo of the current result: https://user-images.githubusercontent.com/29271028/168427643-3a30f34d-a9e0-4ed4-afcb-4d4a628e0a48.mp4
## Documentation 
A good take of intersection and some math comes from [*The game physic cookbook*](https://gamephysicscookbook.com/), but i didn't exactly followed it.

- Row majored matrix
- [Left-handed coordinate system](https://www.evl.uic.edu/ralph/508S98/coordinates.html)  
![left_hand_coord_system](https://user-images.githubusercontent.com/29271028/159712557-554fbb55-8b8e-41b9-852a-567de70e4713.png)  
The rotation order is clockwise (angle positive and looking down the negative axis around which we're rotating).
```
rotation::z_axis(90 degrees) * vec::right = vec::up
```
as well for the cross product orientation result:
```
cross(right, up) = forward
cross(forward, right) = up
cross(up, forward) = right
```
### Method of Separating Axis 
- [Paper explanation, 2D & 3D exemple + computation of collision time using the Method](https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf) or in the doc folder [here](doc/MethodOfSeparatingAxes.pdf)
- The way of implementing the SAT follow this [post](https://dyn4j.org/2010/01/sat/#sat-projshape)

### Quaternion

- compréhension globale - le cours d'Alexandre Meyer [ici](https://perso.liris.cnrs.fr/alexandre.meyer/teaching/master_charanim/aPDF_COURS_M2/M2_1b_Quaternions)
- compréhension globale - euclidean space [ici](https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm)
- l'article de wikipédia [ici](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
- pour la conversion Quaternion/Matrix [ici](https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm)



### Collisions
- short article about impulsions based collision response on [wikidedia](https://en.wikipedia.org/wiki/Collision_response)
- Méthode de a à z : [https://www.cs.cmu.edu/~baraff/sigcourse/](https://www.cs.cmu.edu/~baraff/sigcourse/)

- Calculer les points de contact en 2D pour OBB [ici](https://dyn4j.org/2011/11/contact-points-using-clipping/#cpg-alt)
- Calculer points de contact et impl SAT nécessaire [ici](https://steamcdn-a.akamaihd.net/apps/valve/2015/DirkGregorius_Contacts.pdf)
  
### Améliorations

- a propos des contraintes, objets statiques, stack d'objets etc [ici](https://www.gdcvault.com/play/1020603/Physics-for-Game-Programmers-Understanding)
- Aide pour SAT et calculer les points d'intersections sur les formes primitives [ici](https://steamcdn-a.akamaihd.net/apps/valve/2015/DirkGregorius_Contacts.pdf)
- Clipping sur obb en 2D [ici](https://dyn4j.org/2011/11/contact-points-using-clipping/#cpg-alt)
- amélioration pour SAT [ici](https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The)
- [Quickhull](https://steamcdn-a.akamaihd.net/apps/valve/2014/DirkGregorius_ImplementingQuickHull.pdf)

### Aide
- [global course on physic](https://physics.info/)
- [Application](https://sumo.app/3d/?lang=en) web de visualisation 3D

