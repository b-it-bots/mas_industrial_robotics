# 3D Mesh Workflow
Once you have created your favorite 3D model of a part, the following workflow can be used to create all necessary parts in order to add it to a URDF

## Inventor, SolidWorks, etc.
Export your part as STL or directly as DAE if possible.

## MeshLab
NOTE: the below instructions might be available only in MeshLab version >=1.3.3

1. Create convex hull:

     Filters -> Remeshing, Simpilication and Reconstruction -> Convex Hull (select "re-orient all faces coherently" in the dialog box)

2. If the collision model does not appear correctly later on in RViz use the following command to correct this:

     Filters -> Normals, Curvatures and Orientation -> Invert Faces Orientation

3. In case of inconsistent normal orientation (mesh shows transparent areas) execute this:

     Filters -> Normals, Curvatures and Orientation -> Re-Orient all faces coherently

4. Delete original mesh layer: choose "show layer dialog" in menu bar and delete the respective layer 

5. Export model as .dae file

     File -> Export Mesh As ...