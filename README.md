# Angine
Implementation of a Physics engine in Rust with basic physics simulation and collision.

## Documentation 
A good take of intersection and some math comes from [*The game physic cookbook*](https://gamephysicscookbook.com/), but i didn't exactly followed it.

- Row majored matrix
- [Left-handed coordinate system](https://www.evl.uic.edu/ralph/508S98/coordinates.html)
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

### Aide

- [Application](https://sumo.app/3d/?lang=en) web de visualisation 3D
