#include <octree/Octree.h>
#include <octree/FileIO.h>
#include <gtest/gtest.h>

using namespace octree;

TEST(OctreeTest, findIndex) {
	const Eigen::Vector3d corner1(-2, -2, -2);
	const Eigen::Vector3d corner2( 2,  2,  2);
	Node node(corner1, corner2, NULL, 0, FREE);
	ASSERT_EQ(0, node.findIndex(Eigen::Vector3d(-1, -1, -1)));
	ASSERT_EQ(1, node.findIndex(Eigen::Vector3d( 1, -1, -1)));
	ASSERT_EQ(2, node.findIndex(Eigen::Vector3d(-1,  1, -1)));
	ASSERT_EQ(3, node.findIndex(Eigen::Vector3d( 1,  1, -1)));
	ASSERT_EQ(4, node.findIndex(Eigen::Vector3d(-1, -1,  1)));
	ASSERT_EQ(5, node.findIndex(Eigen::Vector3d( 1, -1,  1)));
	ASSERT_EQ(6, node.findIndex(Eigen::Vector3d(-1,  1,  1)));
	ASSERT_EQ(7, node.findIndex(Eigen::Vector3d( 1,  1,  1)));
}

TEST(OctreeTest, findNode) {
	const Eigen::Vector3d corner1(-2, -2, -2);
	const Eigen::Vector3d corner2( 2,  2,  2);
	const unsigned int maxDepth = 4;
	Octree oc(corner1, corner2, maxDepth);

    Node *node = oc.findNode(Eigen::Vector3d(1.5, -0.5, 0.5));
    ASSERT_FALSE(node == NULL) << "The method returned NULL";
    ASSERT_FALSE(node != oc.root) << "The method should have returned the root node in the case of an empty octree ";

	oc.root->content = MIXED;
	oc.root->children[0] = new Node(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(0, 0, 0), oc.root, 1, FREE);
	oc.root->children[1] = new Node(Eigen::Vector3d( 0, -2, -2), Eigen::Vector3d(2, 0, 0), oc.root, 1, FREE);
	oc.root->children[2] = new Node(Eigen::Vector3d(-2,  0, -2), Eigen::Vector3d(0, 2, 0), oc.root, 1, FREE);
	oc.root->children[3] = new Node(Eigen::Vector3d( 0,  0, -2), Eigen::Vector3d(2, 2, 0), oc.root, 1, FREE);
	oc.root->children[4] = new Node(Eigen::Vector3d(-2, -2,  0), Eigen::Vector3d(0, 0, 2), oc.root, 1, FREE);
	oc.root->children[5] = new Node(Eigen::Vector3d( 0, -2,  0), Eigen::Vector3d(2, 0, 2), oc.root, 1, MIXED);
	oc.root->children[6] = new Node(Eigen::Vector3d(-2,  0,  0), Eigen::Vector3d(0, 2, 2), oc.root, 1, FREE);
	oc.root->children[7] = new Node(Eigen::Vector3d( 0,  0,  0), Eigen::Vector3d(2, 2, 2), oc.root, 1, FREE);

	oc.root->children[5]->children[0] = new Node(Eigen::Vector3d( 0, -2,  0), Eigen::Vector3d(1, -1, 1), oc.root, 1, FREE);
	oc.root->children[5]->children[1] = new Node(Eigen::Vector3d( 1, -2,  0), Eigen::Vector3d(2, -1, 1), oc.root, 1, FREE);
	oc.root->children[5]->children[2] = new Node(Eigen::Vector3d( 0, -1,  0), Eigen::Vector3d(1,  0, 1), oc.root, 1, FREE);
	oc.root->children[5]->children[3] = new Node(Eigen::Vector3d( 1, -1,  0), Eigen::Vector3d(2,  0, 1), oc.root, 1, MIXED);
	oc.root->children[5]->children[4] = new Node(Eigen::Vector3d( 0, -2,  1), Eigen::Vector3d(1, -1, 2), oc.root, 1, FREE);
	oc.root->children[5]->children[5] = new Node(Eigen::Vector3d( 1, -2,  1), Eigen::Vector3d(2, -1, 2), oc.root, 1, FREE);
	oc.root->children[5]->children[6] = new Node(Eigen::Vector3d( 0, -1,  1), Eigen::Vector3d(1,  0, 2), oc.root, 1, FREE);
	oc.root->children[5]->children[7] = new Node(Eigen::Vector3d( 1, -1,  1), Eigen::Vector3d(2,  0, 2), oc.root, 1, FREE);

    oc.root->children[5]->children[3]->children[0] = new Node(Eigen::Vector3d( 1.5, -0.5,  0.5), Eigen::Vector3d(1, -1,   0), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[1] = new Node(Eigen::Vector3d( 1.5, -0.5,  0.5), Eigen::Vector3d(1,  0,   0), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[2] = new Node(Eigen::Vector3d( 1.5, -0.5,  0.5), Eigen::Vector3d(2, -1,   0), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[3] = new Node(Eigen::Vector3d( 1.5, -0.5,  0.5), Eigen::Vector3d(2,  0,   0), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[4] = new Node(Eigen::Vector3d( 1.5, -0.5,    1), Eigen::Vector3d(1, -1, 0.5), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[5] = new Node(Eigen::Vector3d( 1.5, -0.5,    1), Eigen::Vector3d(1,  0, 0.5), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[6] = new Node(Eigen::Vector3d( 1.5, -0.5,    1), Eigen::Vector3d(2, -1, 0.5), oc.root, 1, FREE);
    oc.root->children[5]->children[3]->children[7] = new Node(Eigen::Vector3d( 1.5, -0.5,    1), Eigen::Vector3d(2,  0, 0.5), oc.root, 1, MIXED);

    node = oc.findNode(Eigen::Vector3d(1.75, -0.25, 0.75));
	ASSERT_FALSE(node == NULL) << "The method returned NULL";
	ASSERT_FALSE(node == oc.root->children[5]) << "The method returned a child node, but did not recurse to find leaf node";
    ASSERT_FALSE(node == oc.root->children[5]->children[3]) << "The method returned a child node, it recursed once but it did not recurse to the end until it finds the leaf node";
    ASSERT_TRUE(node == oc.root->children[5]->children[3]->children[7]) << "The method did not return the correct node";
}

TEST(OctreeTest, split) {
	const double epsilon = 1e-5;
	const Eigen::Vector3d corner1(2, 2, 2);
	const Eigen::Vector3d corner2(3, 4, 5);
	const Eigen::Vector3d point(2.25, 3.5, 3);
	Node parent(corner1, Eigen::Vector3d(4, 6, 8), NULL, 0, MIXED);
	Node node(corner1, corner2, &parent, 1, FREE);
	Node *child = node.split(point);
	ASSERT_FALSE(child == NULL) << "The method returned NULL";
	ASSERT_TRUE(node.content == MIXED) << "The method did not set the node content to MIXED";
	for (size_t i = 0; i < 8; ++i) {
		ASSERT_FALSE(node.children[i] == NULL) << "Child " << i << " is NULL.";
		if (i == 2) {
			ASSERT_TRUE(node.children[i]->content == OCCUPIED) << "The method did not mark the child node containing the point as OCCUPIED";
		} else {
			ASSERT_TRUE(node.children[i]->content == FREE) << "The method did not mark the other child nodes as FREE";
		}
		ASSERT_TRUE(node.children[i]->parent == &node) << "The method did not set the parent pointer of the child nodes to the current node.";
		ASSERT_TRUE(node.children[i]->depth == node.depth + 1) << "The method did not set the depth field of the child nodes correctly.";
	}

	ASSERT_TRUE(node.children[2] == child) << "The method returned the wrong child node.";

	const Eigen::Vector3d trueCorner1[8] = {
		Eigen::Vector3d(2.0, 2.0, 2.0),
		Eigen::Vector3d(2.5, 2.0, 2.0),
		Eigen::Vector3d(2.0, 3.0, 2.0),
		Eigen::Vector3d(2.5, 3.0, 2.0),
		Eigen::Vector3d(2.0, 2.0, 3.5),
		Eigen::Vector3d(2.5, 2.0, 3.5),
		Eigen::Vector3d(2.0, 3.0, 3.5),
		Eigen::Vector3d(2.5, 3.0, 3.5),
	};

	const Eigen::Vector3d trueCorner2[8] = {
		Eigen::Vector3d(2.5, 3.0, 3.5),
		Eigen::Vector3d(3.0, 3.0, 3.5),
		Eigen::Vector3d(2.5, 4.0, 3.5),
		Eigen::Vector3d(3.0, 4.0, 3.5),
		Eigen::Vector3d(2.5, 3.0, 5.0),
		Eigen::Vector3d(3.0, 3.0, 5.0),
		Eigen::Vector3d(2.5, 4.0, 5.0),
		Eigen::Vector3d(3.0, 4.0, 5.0)
	};

	// Check corners
	for (size_t i = 0; i < 8; ++i) {
		const Eigen::Vector3d corner1 = node.children[i]->corner1.cwiseMin(node.children[i]->corner2);
		const Eigen::Vector3d corner2 = node.children[i]->corner1.cwiseMax(node.children[i]->corner2);
		ASSERT_TRUE(corner1.isApprox(trueCorner1[i], epsilon)) << "children[" << i << "]->corner1 is incorrect.";
		ASSERT_TRUE(corner2.isApprox(trueCorner2[i], epsilon)) << "children[" << i << "]->corner2 is incorrect.";
	}
}

/**
 * \brief Node subclass for tracing "delete" calls
 */
class NodeTest : public Node {
public:
	bool& destructorCalled;
	NodeTest(const Eigen::Vector3d& corner1, const Eigen::Vector3d& corner2, Node * const parent, const unsigned int depth, const Content content, bool& destructorCalled)
	: Node(corner1, corner2, parent, depth, content), destructorCalled(destructorCalled) {
		destructorCalled = false;
	}
	~NodeTest() {
		destructorCalled = true;
	}
};

TEST(OctreeTest, merge) {
	const Eigen::Vector3d corner1(-2, -2, -2);
	const Eigen::Vector3d corner2( 2,  2,  2);

	// Test case: different labels --> cannot merge
	{
		Node node(corner1, corner2, NULL, 0, MIXED);
		bool destructorCalled[8];
		node.children[0] = new NodeTest(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(0, 0, 0), &node, 1, FREE, destructorCalled[0]);
		node.children[1] = new NodeTest(Eigen::Vector3d( 0, -2, -2), Eigen::Vector3d(2, 0, 0), &node, 1, FREE, destructorCalled[1]);
		node.children[2] = new NodeTest(Eigen::Vector3d(-2,  0, -2), Eigen::Vector3d(0, 2, 0), &node, 1, FREE, destructorCalled[2]);
		node.children[3] = new NodeTest(Eigen::Vector3d( 0,  0, -2), Eigen::Vector3d(2, 2, 0), &node, 1, FREE, destructorCalled[3]);
		node.children[4] = new NodeTest(Eigen::Vector3d(-2, -2,  0), Eigen::Vector3d(0, 0, 2), &node, 1, OCCUPIED, destructorCalled[4]);
		node.children[5] = new NodeTest(Eigen::Vector3d( 0, -2,  0), Eigen::Vector3d(2, 0, 2), &node, 1, FREE, destructorCalled[5]);
		node.children[6] = new NodeTest(Eigen::Vector3d(-2,  0,  0), Eigen::Vector3d(0, 2, 2), &node, 1, FREE, destructorCalled[6]);
		node.children[7] = new NodeTest(Eigen::Vector3d( 0,  0,  0), Eigen::Vector3d(2, 2, 2), &node, 1, FREE, destructorCalled[7]);
		const bool merged = node.merge();
		ASSERT_FALSE(merged) << "The method returned true even though the children cannot be merged.";
		for (size_t i = 0; i < 8; ++i) {
			ASSERT_TRUE(node.children[i]) << "The method removed a child node even though the children cannot be merged.";
		}
		for (size_t i = 0; i < 8; ++i) {
			ASSERT_FALSE(destructorCalled[i]) << "The method deleted a child node even though the children cannot be merged.";
		}
	}

	// Test case: all free --> merge to free
	{
		Node node(corner1, corner2, NULL, 0, MIXED);
		bool destructorCalled[8];
		node.children[0] = new NodeTest(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(0, 0, 0), &node, 1, FREE, destructorCalled[0]);
		node.children[1] = new NodeTest(Eigen::Vector3d( 0, -2, -2), Eigen::Vector3d(2, 0, 0), &node, 1, FREE, destructorCalled[1]);
		node.children[2] = new NodeTest(Eigen::Vector3d(-2,  0, -2), Eigen::Vector3d(0, 2, 0), &node, 1, FREE, destructorCalled[2]);
		node.children[3] = new NodeTest(Eigen::Vector3d( 0,  0, -2), Eigen::Vector3d(2, 2, 0), &node, 1, FREE, destructorCalled[3]);
		node.children[4] = new NodeTest(Eigen::Vector3d(-2, -2,  0), Eigen::Vector3d(0, 0, 2), &node, 1, FREE, destructorCalled[4]);
		node.children[5] = new NodeTest(Eigen::Vector3d( 0, -2,  0), Eigen::Vector3d(2, 0, 2), &node, 1, FREE, destructorCalled[5]);
		node.children[6] = new NodeTest(Eigen::Vector3d(-2,  0,  0), Eigen::Vector3d(0, 2, 2), &node, 1, FREE, destructorCalled[6]);
		node.children[7] = new NodeTest(Eigen::Vector3d( 0,  0,  0), Eigen::Vector3d(2, 2, 2), &node, 1, FREE, destructorCalled[7]);
		const bool merged = node.merge();
		ASSERT_TRUE(merged) << "The method returned false even though all children are FREE and can be merged.";
		ASSERT_FALSE(node.content == MIXED) << "The method did not change the content label of the node from MIXED to the common label.";
		ASSERT_TRUE(node.content == FREE) << "The method did not change the content label to FREE although all child nodes are marked as FREE.";
		for (size_t i = 0; i < 8; ++i) {
			ASSERT_TRUE(destructorCalled[i]) << "The method did not delete the children by calling 'delete children[i]'.";
		}
		for (size_t i = 0; i < 8; ++i) {
			ASSERT_FALSE(node.children[i]) << "The method did not set the children to NULL.";
		}
	}

	// Test case: all occupied --> merge to occupied
	{
		Node node(corner1, corner2, NULL, 0, MIXED);
		bool destructorCalled[8];
		node.children[0] = new NodeTest(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(0, 0, 0), &node, 1, OCCUPIED, destructorCalled[0]);
		node.children[1] = new NodeTest(Eigen::Vector3d( 0, -2, -2), Eigen::Vector3d(2, 0, 0), &node, 1, OCCUPIED, destructorCalled[1]);
		node.children[2] = new NodeTest(Eigen::Vector3d(-2,  0, -2), Eigen::Vector3d(0, 2, 0), &node, 1, OCCUPIED, destructorCalled[2]);
		node.children[3] = new NodeTest(Eigen::Vector3d( 0,  0, -2), Eigen::Vector3d(2, 2, 0), &node, 1, OCCUPIED, destructorCalled[3]);
		node.children[4] = new NodeTest(Eigen::Vector3d(-2, -2,  0), Eigen::Vector3d(0, 0, 2), &node, 1, OCCUPIED, destructorCalled[4]);
		node.children[5] = new NodeTest(Eigen::Vector3d( 0, -2,  0), Eigen::Vector3d(2, 0, 2), &node, 1, OCCUPIED, destructorCalled[5]);
		node.children[6] = new NodeTest(Eigen::Vector3d(-2,  0,  0), Eigen::Vector3d(0, 2, 2), &node, 1, OCCUPIED, destructorCalled[6]);
		node.children[7] = new NodeTest(Eigen::Vector3d( 0,  0,  0), Eigen::Vector3d(2, 2, 2), &node, 1, OCCUPIED, destructorCalled[7]);
		const bool merged = node.merge();
		ASSERT_TRUE(merged) << "The method returned false even though all children are FREE and can be merged.";
		ASSERT_FALSE(node.content == MIXED) << "The method did not change the content label of the node from MIXED to the common label.";
		ASSERT_TRUE(node.content == OCCUPIED) << "The method did not change the content label to OCCUPIED although all child nodes are marked as OCCUPIED.";
		for (size_t i = 0; i < 8; ++i) {
			ASSERT_TRUE(destructorCalled[i]) << "The method did not delete the children by calling 'delete children[i]'.";
		}
		for (size_t i = 0; i < 8; ++i) {
			ASSERT_FALSE(node.children[i]) << "The method did not set the children to NULL.";
		}
	}

	// Test case: all mixed --> cannot merge
	{
		Node node(corner1, corner2, NULL, 0, MIXED);
		bool destructorCalled[8];
		node.children[0] = new NodeTest(Eigen::Vector3d(-2, -2, -2), Eigen::Vector3d(0, 0, 0), &node, 1, MIXED, destructorCalled[0]);
		node.children[1] = new NodeTest(Eigen::Vector3d( 0, -2, -2), Eigen::Vector3d(2, 0, 0), &node, 1, MIXED, destructorCalled[1]);
		node.children[2] = new NodeTest(Eigen::Vector3d(-2,  0, -2), Eigen::Vector3d(0, 2, 0), &node, 1, MIXED, destructorCalled[2]);
		node.children[3] = new NodeTest(Eigen::Vector3d( 0,  0, -2), Eigen::Vector3d(2, 2, 0), &node, 1, MIXED, destructorCalled[3]);
		node.children[4] = new NodeTest(Eigen::Vector3d(-2, -2,  0), Eigen::Vector3d(0, 0, 2), &node, 1, MIXED, destructorCalled[4]);
		node.children[5] = new NodeTest(Eigen::Vector3d( 0, -2,  0), Eigen::Vector3d(2, 0, 2), &node, 1, MIXED, destructorCalled[5]);
		node.children[6] = new NodeTest(Eigen::Vector3d(-2,  0,  0), Eigen::Vector3d(0, 2, 2), &node, 1, MIXED, destructorCalled[6]);
		node.children[7] = new NodeTest(Eigen::Vector3d( 0,  0,  0), Eigen::Vector3d(2, 2, 2), &node, 1, MIXED, destructorCalled[7]);
		const bool merged = node.merge();
		ASSERT_FALSE(merged) << "The method merged child nodes that have MIXED content, which cannot be merged.";
	}
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
