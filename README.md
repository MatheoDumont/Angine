# Engine
Implementation of a Physics engine in Rust.

## Documentation 
A good take of intersection and some math comes from [*The game physic cookbook*](https://gamephysicscookbook.com/), but i didn't exactly followed it.

- Row majored matrix
- [Left-handed coordinate system](https://www.evl.uic.edu/ralph/508S98/coordinates.html)
### Method of Separating Axis 
- [Paper explanation, 2D & 3D exemple + computation of collision time using the Method](https://www.geometrictools.com/Documentation/MethodOfSeparatingAxes.pdf) or in the doc folder [here](doc/MethodOfSeparatingAxes.pdf)
- Th way of implementing the SAT follow this [post](https://dyn4j.org/2010/01/sat/#sat-projshape)

### Collisions
Simplifiés de telle sorte que on intègre la position au temps t+1 comme étant  :
```
p_t+1 = p_t + v_t+1 * dt
```

Donc entre t et t+1, la vitesse est constante. On détecte la collision dans cette interval. Si la vitesse d'un objet est suffisante pour dépasser un autre dans un temps inférieur au dt = t+1 - t, alors la collision ne sera pas détectée. On peut clamp la vitesse pour empêcher cela.
Cette technique est utilisée pour rester temps réel.
