#ifndef OCTREE_H_
#define OCTREE_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace octree {

/**
 * \brief Enumeration constants for the content of a octree cube
 */
enum Content {
	OCCUPIED,    ///< The cube is completely occupied
	FREE,        ///< The cube is completely free
	MIXED        ///< The cube has both free and occupied child nodes
};

/**
 * \brief Element of an octree.
 *
 * This class represents a node of an octree, which can also be visualized as a cube.
 */
class Node {
public:
	const Eigen::Vector3d corner1;           ///< Coordinates of the first corner of the cube.
	const Eigen::Vector3d corner2;           ///< Coordinates of the opposite corner of the cube.
	Node * const parent;                     ///< Parent node within the tree (NULL = the node is already the root node)
	const unsigned int depth;                ///< Depth of the node within the tree (0 = root node)
	Content content;                         ///< Content label of the node (FREE, OCCUPIED, or MIXED).
	Node * children[8];                      ///< 8 children = subcubes or octants of the node (all NULL = node is a leaf)

	/**
	 * \brief Constructor
	 * \param[in] corner1 Coordinates of the first corner of the cube
	 * \param[in] corner2 Coordinates of the opposite corner of the cube
	 * \param[in] parent Pointer to the parent node within the tree (NULL = this node is the root node)
	 * \param[in] depth Depth of the node within the tree (0 = this node is the root node)
	 * \param[in] content Content label of the node (FREE, OCCUPIED, or MIXED)
	 *
	 * The children pointers will be initialized to NULL.
	 */
	Node (const Eigen::Vector3d& corner1, const Eigen::Vector3d& corner2, Node * const parent, const unsigned int depth, const Content content)
	: corner1(corner1), corner2(corner2), parent(parent), depth(depth), content(content) {
		// Initialize the children to NULL
		for (size_t i = 0; i < 8; ++i) {
			children[i] = NULL;
		}
	}

	/**
	 * \brief Destructor.
	 *
	 * Deletes all children.
	 */
	virtual ~Node() {
		for (size_t i = 0; i < 8; ++i) {
			if (children[i]) {
				delete children[i];
				children[i] = NULL;
			}
		}
	}

	unsigned int findIndex(const Eigen::Vector3d& point) const;
	Node* split(const Eigen::Vector3d& point);
	bool merge();

};

/**
 * \brief Octree representing a 3D environment.
 */
class Octree {
public:
	const unsigned int maxDepth;   ///< Maximum depth of the tree, limiting the minimum cube size
	Node * const root;             ///< Root node of the tree

	/**
	 * \brief Constructor
	 * \param[in] corner1 Coordinates of the first corner of the root cube
	 * \param[in] corner2 Coordinates of the opposite corner of the root cube
	 * \param[in] maxDepth Maximum depth of the tree
	 */
	Octree(const Eigen::Vector3d& corner1, const Eigen::Vector3d& corner2, const unsigned int maxDepth)
	: maxDepth(maxDepth), root(new Node(corner1, corner2, NULL, 0, FREE)) {
	}

	/**
	 * \brief Destructor
	 *
	 * Deletes the root node (and recursively all tree nodes).
	 */
	~Octree() {
		if (root) {
			delete root;
		}
	}

	bool insertPoint(const Eigen::Vector3d& point);
	Node * findNode(const Eigen::Vector3d& point) const;
};


}  // namespace octree


#endif  // OCTREE_H_
