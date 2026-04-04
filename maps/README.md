Place the saved house map from `map_saver` in this directory.

Expected filenames:

- `house_map.yaml`
- `house_map.pgm`

You can also keep the files anywhere else and pass the absolute YAML path to:

```bash
roslaunch cs603_particle_filter particle_filter.launch map_yaml:=/absolute/path/to/house_map.yaml
```
