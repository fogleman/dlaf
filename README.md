# dlaf

Diffusion-limited aggregation, fast. Diffusion-limited as f*ck.

### Features

- 2D or 3D diffusion-limited aggregation
- Super fast? 35 seconds to compute one million points on a single core.

### Dependencies

- [boost](https://www.boost.org/) (used for its spatial index)

### Usage

```bash
git clone https://github.com/fogleman/dlaf.git
cd dlaf
make
./dlaf > output.csv
```

### Output Format

The `parent_id` tells you which particle was joined to. It is -1 for initial seed positions.
When 2D is used, Z will be zero for all points.

```bash
# columns are: id, parent_id, x, y, z
$ head output.csv
0,-1,0,0,0
1,0,0.934937,0.354814,0
2,0,0.0525095,-0.99862,0
3,1,0.989836,1.35331,0
4,3,1.92472,1.70826,0
5,3,0.65572,2.29584,0
6,4,2.90818,1.88937,0
7,0,-0.989205,0.146538,0
8,2,0.917631,-1.50018,0
9,5,0.832028,3.28017,0
```

### Rendering

Rendering is left to you. This code just gives you the location of the points and their hierarchy.
But here is an example rendering in 2D with one million particles:

![Example](https://i.imgur.com/Ma1hv3z.png)

### Hooks & Parameters

The code implements a standard diffusion-limited aggregation algorithm. But there are several parameters and code hooks that let you tweak its behavior.

The following parameters can be set on a `Model` instance.

| Parameter | Description |
| --- | --- |
| `AttractionDistance` | Defines how close together particles must be in order to join together. |
| `ParticleSpacing` | Defines the distance between particles when they become joined together. |
| `MinMoveDistance` | Defines the minimum distance that a particle will move in an iteration during its random walk. |
| `Stubbornness` | Defines how many join attempts must occur before a particle will allow another particle to join to it. |
| `Stickiness` | Defines the probability that a particle will allow another particle to join to it. |

The following hooks allow you to define the algorithm behavior in small, well-defined functions.

| Hook | Description |
| --- | --- |
| `RandomStartingPosition()` | Returns a starting position for a new particle to begin its random walk. |
| `ShouldReset(p)` | Returns true if the particle has gone too far away and should be reset to a new random starting position. |
| `ShouldJoin(p, parent)` | Returns true if the point should attach to the specified parent particle. This is only called when the point is already within the required attraction distance. If false is returned, the particle will continue its random walk instead of joining to the other particle. |
| `PlaceParticle(p, parent)` | Returns the final placement position of the particle. |
| `MotionVector(p)` | Returns a vector specifying the direction that the particle should move for one iteration. The distance that it will move is determined by the algorithm. |
