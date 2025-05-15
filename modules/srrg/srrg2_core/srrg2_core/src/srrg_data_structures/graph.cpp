#include "graph.h"
#include <cassert>

namespace srrg2_core {

  GraphBase::ElementBase::~ElementBase() {}
  
  GraphBase::VertexBase::VertexBase(GraphBase* graph_):
    ElementBase(graph_)
  {}

  GraphBase::VertexBase::~VertexBase() {
    for (auto e: edges()){

      if (e->_from == this)
        e->_from=0;

      if (e->_to == this)
        e->_to=0;
    }
  }

  GraphBase::EdgeBase::EdgeBase(VertexBase* from_,
                                VertexBase* to_,
                                GraphBase* graph_):
    ElementBase(graph_),
    _from(from_),
    _to(to_)
  {}


  GraphBase::EdgeBase::~EdgeBase() {
    if (_from) {
      auto it = _from->edges().find(this);
      _from->edges().erase(it);
    }
    if (_to) {
      auto it = _to->edges().find(this);
      _from->edges().erase(it);
    }
  }

  bool GraphBase::isVertex(const VertexBasePtr& v) const {
    return _vertices.find(v)!=_vertices.end();
  }

  bool GraphBase::isEdge(const EdgeBasePtr& e) const {
    return _edges.find(e)!=_edges.end();
  }

  void GraphBase::addVertex(VertexBasePtr& v) {
    assert(! v->graph() && "vertex not detached");
    assert(! isVertex(v) && "vertex already in graph");
    _vertices.insert(v);
  }

  void GraphBase::detachVertex(VertexBasePtr& v) {
    assert(v->graph()==this && "vertex's graph not correct");
    auto it=_vertices.find(v);
    assert(it!=_vertices.end() && "vertex not in graph");
    _vertices.erase(it);
    v->_graph=0;
  }

  void GraphBase::addEdge(EdgeBasePtr& e) {
    assert(! isEdge(e) && "edge already in graph");
    assert(e->graph()==0 && "edge not detached");
    VertexBase* v_to=e->to();
    if (v_to) {
      v_to->edges().insert(e.get());
    }
    VertexBase* v_from=e->from();
    if (v_from && v_from!=v_to) {
      v_from->edges().insert(e.get());
    }
  }

  // detaches an edge from the graph.
  // exception if e not in the graph
  void GraphBase::detachEdge(EdgeBasePtr& e) {
    assert(e->_graph == this && "edge's graph is not set");
    auto e_it=_edges.find(e);
    assert(e_it!=_edges.end() && "edge not in graph");
    _edges.erase(e_it);
  }

}
