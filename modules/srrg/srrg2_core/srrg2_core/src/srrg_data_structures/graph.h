#pragma once
#include <set>
#include <memory>

namespace srrg2_core {
  /* This class defines an abstract graph supporting
     insertion and detaching of vertices and edges
     Each vertex has a list of edges.

     Detaching results in disconnecting an item from the
     graph and setting its owner to 0.
     All connected structures that would be made inconsistent upon
     detach are detached too
*/


  class GraphBase {
  public:

    /**
       Base element of a graph. It only has an owner;
     */
    class ElementBase {
      friend class GraphBase;


      // what if an element belongs to more than one graph?
    public:
      //! constructs a graph element
      ElementBase(GraphBase* graph_): _graph(graph_) {}

      //! returns the owning graph
      inline GraphBase* graph() { return _graph; }

      inline const GraphBase* graph() const { return _graph; }

      virtual ~ElementBase();
    protected:
      GraphBase* _graph;
    };

    class EdgeBase;
    class VertexBase;

    using VertexBasePtr      = std::shared_ptr<VertexBase>;
    using EdgeBasePtr        = std::shared_ptr<EdgeBase>;
    using VertexBasePtrSet   = std::set<VertexBasePtr >;
    using EdgeBasePtrSet     = std::set<EdgeBasePtr >;
    
    //! base vertex of the graph
    //! it has a set of *leaving and entering* edges
    class VertexBase: public ElementBase {
      friend class GraphBase;
    public:
      VertexBase(GraphBase* graph_ = 0);

      virtual ~VertexBase();
      inline const std::set<EdgeBase*>& edges() const {return _edges;}
      inline std::set<EdgeBase*>& edges() {return _edges;}
    protected:
      std::set<EdgeBase*> _edges;
    };

    //! base edge of a graph
    //! an edge connects two existing vertices in the graph
    class EdgeBase : public ElementBase {
      friend class VertexBase;
      friend class GraphBase;
    public:
      EdgeBase(VertexBase* from_,
               VertexBase* to_,
               GraphBase* graph_ = 0);

      // vertex from where the edge leaves
      inline VertexBase* from() { return _from; }

      inline const VertexBase* from() const  { return _from; }

      // vertex to which the edge leads
      inline VertexBase* to() { return _to; }

      inline const VertexBase* to() const { return _to; }
      
      virtual ~EdgeBase();
    protected:
      VertexBase* _from;
      VertexBase* _to;
    };

    inline VertexBasePtrSet& vertices() { return _vertices; }
    inline const VertexBasePtrSet& vertices() const { return _vertices; }
    inline EdgeBasePtrSet& edges() { return _edges; }
    inline const EdgeBasePtrSet& edges() const { return _edges; }

    // true if vertex in graph
    bool isVertex(const VertexBasePtr& v) const;

    // true if edge in graph
    bool isEdge(const EdgeBasePtr& e) const;
    
    // adds a vertex to the graph.
    // v->edges should be empty and the graph should be set to null
    // throws an exception on error
    void addVertex(VertexBasePtr& v);
    
    // detaches a vertex from the graph
    // exception if vertex not present
    // it sets v to 0 to all connected edges
    void detachVertex(VertexBasePtr& v);
    
    // adds an edge to the graph
    // exception if the connected vertices do not belong to the graph
    // side effect on the edge set in the vertices, to add the new edge
    void addEdge(EdgeBasePtr& e);
    
    // detaches an edge from the graph.
    // exception if e not in the graph
    void detachEdge(EdgeBasePtr& e);

  protected:
    VertexBasePtrSet _vertices;
    EdgeBasePtrSet _edges;

  };

}
