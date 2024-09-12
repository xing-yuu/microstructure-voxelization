# Microstructure Voxelization
An easy-to-use command-line tool to convert microstructure meshes to voxel grids (For property calculation).
## Requires
- [numpy](http://www.numpy.org/)
- [open3D](https://www.open3d.org/)
## Usage
Program options:
 * `--mesh <Microstructure model file>`: **(required)** A path to the polygon-based 3D Microstructure file. 
 * `--mode <The voxelization mode>`: **(default: solid)**   Surface and solid mode.
 * `--res <The resolution of voxels>`: **(default: 64)**   The resolution of voxels.
 * `--visualization <Whether need to visualize voxels>`: **(default: True)**

### Example
```sh
python voxel.py --mesh=S-T-0-1-2.obj --mode=solid
```