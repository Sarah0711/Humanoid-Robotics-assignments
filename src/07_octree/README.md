# Exercise 7: Octrees

Assume that we have a point cloud of a 3D environment, for example 
retrieved from a laser scanner or a 3D camera. Now we would like to 
represent this data as an octree.

The octree algorithm starts with a complete cube and subdivides it 
recursively into eight smaller cubes. It only splits cubes that are 
neither fully occupied nor fully empty until all cubes are either occupied 
or free. When inserting a new point, you keep splitting the cubes until you 
reach a pre-defined minimum size of a 3D cube, i.e., a certain depth within
the octree. The order of the subcubes is given in the figure on the
exercise sheet.

In the source code for this exercise, we represent a 3D cube as an instance 
of class `Node`. Each node has a pointer to the parent node and 8 pointers 
to child nodes for representing the tree structure.

**Exercise steps:**
1. The method `Octree::insertPoint(const Eigen::Vector3d& point)` is 
   already given in the code and contains the general algorithm for 
   inserting a new point into an octree. Inspect the code to find out how 
   the algorithm works.
2. Implement `Node::findIndex(const Eigen::Vector3d& point)`, which is a member 
   function of class `Node`, and is responsible for returning the index (0-7)
   of the child cube that will contain the given input `point` if we split
   that node (Note: Please refer to the figure provided in the sheet to find out how
   these indices are assigned).
3. Implement `Octree::findNode(const Eigen::Vector3d& point)`, which is a member of
   class `Octree` and traverses the tree from the root to find existing the 
   existing leaf `Node` that contains a given point.
4. Implement `Node::split(const Eigen::Vector3d& point)` which is a member 
   function of class `Node`, and splits it to eight child nodes. It returns the 
   child node containing the given point, and sets its status temporarily to `OCCUPIED`. 
   The status of the other empty child node are set to `FREE`. The status of the parent 
   node is set to `MIXED`, i.e. neither completely occupied nor completely free. 
5. Implement `Node::merge()` which tries to merge the current node in case all children 
   are either occupied or free. It sets the status of the parent to the common status
   of the children and deletes all the children nodes.
   
The `scripts` folder contains a Gnuplot script for rendering an interactive 3D view 
of the octree that your program generates. You can move the 3D view with your mouse. 
Alternatively, you will find a top-down view and a rotating 3D animation in the Wiki.
