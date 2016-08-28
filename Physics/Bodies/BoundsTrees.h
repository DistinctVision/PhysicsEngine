#ifndef PE_BOUNDSTREES_H
#define PE_BOUNDSTREES_H

#include <vector>
#include "Shape.h"

namespace PE {

class BoundsTree
{
public:
    struct Node {
        Shape* shape;
        Bounds bounds;
        int indexA;
        int indexB;
    };

    bool isEmpty() const;

    int countNodes() const;
    const Node& rootNode() const;
    const Node& node(int index) const;
    Node& rootNode();
    Node& node(int index);

    void compute(const std::vector<Shape*>& shapes);

    void update();

private:
    std::vector<Node> m_nodes;

    void _computeNode(size_t indexNode, const std::vector<Shape*>& shapes);
    Bounds _computeBounds(std::size_t indexNode);
};

} // namespace PE

#endif // PE_BOUNDSTREES_H
