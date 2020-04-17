# PCL Tutorials

This repository contains some of the files which were used to follow through the Point Cloud Library (PCL) tutorials.

See [here](http://www.pointclouds.org/documentation/) for more information regarding PCL documentation and tutorials.

To visualise the outputs, either use the built in viewer class/functions or install and use `pcl_viewer`. PCL viewer can be installed by

```
sudo apt install pcl-tools
```

Then if you want to visualise `example.pcd`, you can do `pcl_viewer example.pcd`. See [here](http://pointclouds.org/documentation/tutorials/walkthrough.php#binaries) for more information.

## Cluster Segmented

The following input seems to work well for the limited test case `./cluster_segmented 0.08 10 300`

This corresponds to:

* Tolerance = 8cm
* Min points = 10
* Max points = 300
