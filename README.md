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

## Documentation 
A good take of intersection and some math comes from [*The game physic cookbook*](https://gamephysicscookbook.com/), but i didn't exactly followed it.

- Row majored matrix
- [Left-handed coordinate system](https://www.evl.uic.edu/ralph/508S98/coordinates.html)  
![left_hand_coord_system](https://user-images.githubusercontent.com/29271028/159712557-554fbb55-8b8e-41b9-852a-567de70e4713.png)  
The rotation order is clockwise 
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
Simplifiés de telle sorte que on intègre la position au temps t+1 comme étant  :
```
p_t+1 = p_t + v_t+1 * dt
```

Donc entre t et t+1, la vitesse est constante. On détecte la collision dans cette interval. Si la vitesse d'un objet est suffisante pour dépasser un autre dans un temps inférieur au dt = t+1 - t, alors la collision ne sera pas détectée. On peut clamp la vitesse pour empêcher cela.
Cette technique est utilisée pour rester temps réel.

- Calculer les points de contact en 2D pour OBB [ici](https://dyn4j.org/2011/11/contact-points-using-clipping/#cpg-alt)
- Calculer points de contact et impl SAT nécessaire [ici](https://steamcdn-a.akamaihd.net/apps/valve/2015/DirkGregorius_Contacts.pdf)
### Idées

- Après une broadphase, calcul le temps exact de collision en se basant sur la vitesse linéaire et angulaire des objets.

- Aide pour SAT et calculer les points d'intersections sur les formes primitives [ici](https://steamcdn-a.akamaihd.net/apps/valve/2015/DirkGregorius_Contacts.pdf)
- Clipping sur obb en 2D [ici](https://dyn4j.org/2011/11/contact-points-using-clipping/#cpg-alt)

### Améliorations
- amélioration pour SAT [ici](https://www.gdcvault.com/play/1017646/Physics-for-Game-Programmers-The)
- [Quickhull](https://steamcdn-a.akamaihd.net/apps/valve/2014/DirkGregorius_ImplementingQuickHull.pdf)

### Aide

- [Application](https://sumo.app/3d/?lang=en) web de visualisation 3D

