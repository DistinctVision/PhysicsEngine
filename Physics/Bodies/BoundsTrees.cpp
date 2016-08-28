#include "BoundsTrees.h"
#include <cassert>

namespace PE {

bool BoundsTree::isEmpty() const
{
    return m_nodes.empty();
}

int BoundsTree::countNodes() const
{
    return (int)m_nodes.size();
}

const BoundsTree::Node& BoundsTree::rootNode() const
{
    return m_nodes[0];
}

const BoundsTree::Node& BoundsTree::node(int index) const
{
    return m_nodes[index];
}

BoundsTree::Node& BoundsTree::rootNode()
{
    return m_nodes[0];
}

BoundsTree::Node& BoundsTree::node(int index)
{
    return m_nodes[index];
}

void BoundsTree::compute(const std::vector<Shape*>& shapes)
{
    if (shapes.empty())
        return;
    m_nodes.resize(1);
    _computeNode(0, shapes);
}

void BoundsTree::update()
{
    if (m_nodes.empty())
        return;
    _computeBounds(0);
}

void BoundsTree::_computeNode(std::size_t indexNode, const std::vector<Shape*>& shapes)
{
    if (shapes.size() <= 1) {
        if (shapes.size() == 0) {
            m_nodes[indexNode].shape = nullptr;
            m_nodes[indexNode].bounds.init();
            assert(false);
        } else {
            m_nodes[indexNode].shape = shapes[0];
            m_nodes[indexNode].bounds = m_nodes[indexNode].shape->getLocalBounds();
        }
        return;
    }
    m_nodes[indexNode].bounds.init();
    Bounds* localBounds = new Bounds[shapes.size()];
    Vector3 center(0.0f, 0.0f, 0.0f);
    std::size_t i;
    for (i = 0; i < shapes.size(); ++i) {
        localBounds[i] = shapes[i]->getLocalBounds();
        m_nodes[indexNode].bounds.min.minAxis(localBounds[i].min);
        m_nodes[indexNode].bounds.max.maxAxis(localBounds[i].max);
        center += localBounds[i].getCenter();
    }
    Vector3 d = m_nodes[indexNode].bounds.max - m_nodes[indexNode].bounds.min;
    int indexMaxAxis = (d.x > d.y) ? 0 : 1;
    if (d.z > d[indexMaxAxis])
        indexMaxAxis = 2;
    float center_x = center[indexMaxAxis];
    std::vector<Shape*> shapesA;
    std::vector<Shape*> shapesB;
    for (i = 0; i < shapes.size(); ++i) {
        if (localBounds[i].getCenter()[indexMaxAxis] < center_x)
            shapesA.push_back(shapes[i]);
        else
            shapesB.push_back(shapes[i]);
    }
    delete[] localBounds;
    m_nodes.resize(m_nodes.size() + 2);
    m_nodes[indexNode].indexA = m_nodes.size() - 2;
    m_nodes[indexNode].indexB = m_nodes.size() - 1;
    m_nodes[indexNode].shape = nullptr;
    if (shapesA.empty()) {
        shapesA.push_back(*(shapesB.begin() + (shapesB.size() - 1)));
        shapesB.resize(shapesB.size() - 1);
    } else if (shapesB.empty()) {
        shapesB.push_back(*(shapesA.begin() + (shapesA.size() - 1)));
        shapesA.resize(shapesA.size() - 1);
    }
    _computeNode(m_nodes[indexNode].indexA, shapesA);
    _computeNode(m_nodes[indexNode].indexB, shapesB);
}

Bounds BoundsTree::_computeBounds(size_t indexNode)
{
    if (m_nodes[indexNode].shape != nullptr) {
        m_nodes[indexNode].bounds = m_nodes[indexNode].shape->bounds();
        return m_nodes[indexNode].bounds;
    }
    m_nodes[indexNode].bounds = _computeBounds(m_nodes[indexNode].indexA);
    m_nodes[indexNode].bounds.merge(_computeBounds(m_nodes[indexNode].indexB));
    return m_nodes[indexNode].bounds;
}

}

