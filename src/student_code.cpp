#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> inter_points;
    for (int i = 0; i < points.size() - 1; i++) {
      inter_points.push_back((1 - t) * points[i] + t * points[i + 1]);
    }
    return inter_points;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> inter_points;
    for (int i = 0; i < points.size() - 1; i++) {
      inter_points.push_back((1 - t) * points[i] + t * points[i + 1]);
    }
    return inter_points;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    if (points.size() == 1) {
      return points[0];
    }
    return evaluate1D(evaluateStep(points, t), t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> movingPoints;
    for (int i = 0; i < controlPoints.size(); i++) {
      movingPoints.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(movingPoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    Vector3D avgNormal(0, 0, 0);
    
    HalfedgeCIter h = this->halfedge();
    Vector3D v = this->position;
    do {
      Vector3D v1 = h->next()->vertex()->position;
      Vector3D v2 = h->next()->next()->vertex()->position;
      //avgNormal += cross(v1 - v, v2 - v);
      avgNormal += cross(v1 - v, v2 - v);
      h = h->twin()->next();
    } while (h != this->halfedge());
    return avgNormal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    // return if e0 has no twin
    if (e0->isBoundary()) {
      return e0;
    }
    // collect elements
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();

    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();

    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // reassign elements
    h0->setNeighbors(h1, h3, v3, e0, f0);
    h1->setNeighbors(h2, h7, v2, e2, f0);
    h2->setNeighbors(h0, h8, v0, e3, f0);
    h3->setNeighbors(h4, h0, v2, e0, f1);
    h4->setNeighbors(h5, h9, v3, e4, f1);
    h5->setNeighbors(h3, h6, v1, e1, f1);
    h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());

    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h3;
    v3->halfedge() = h0;

    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    
    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    // collect elements
    // view b = m
    // inner triangle
    HalfedgeIter m_c = e0->halfedge();
    HalfedgeIter c_a = m_c->next();
    HalfedgeIter a_m = c_a->next();
    HalfedgeIter c_m = m_c->twin();
    HalfedgeIter m_d = c_m->next();
    HalfedgeIter d_c = m_d->next();
    // outer triangle
    HalfedgeIter a_c = c_a->twin();
    HalfedgeIter c_d = d_c->twin();
    HalfedgeIter d_b = m_d->twin();
    HalfedgeIter b_a = a_m->twin();

    VertexIter a = a_m->vertex();
    VertexIter b = m_d->vertex();
    VertexIter c = c_a->vertex();
    VertexIter d = d_c->vertex();

    EdgeIter E_m_c = e0;
    EdgeIter E_c_a = c_a->edge();
    EdgeIter E_a_b = a_m->edge();
    EdgeIter E_b_d = m_d->edge();
    EdgeIter E_d_c = d_c->edge();

    FaceIter F_c_a_m = m_c->face();
    FaceIter F_c_m_d = c_m->face();

    // allocate new elements
    HalfedgeIter m_a = newHalfedge();
    HalfedgeIter a_b = newHalfedge();
    HalfedgeIter b_m = newHalfedge();
    HalfedgeIter d_m = newHalfedge();
    HalfedgeIter m_b = newHalfedge();
    HalfedgeIter b_d = newHalfedge();

    VertexIter m = newVertex();

    EdgeIter E_a_m = newEdge();
    EdgeIter E_b_m = newEdge();
    EdgeIter E_m_d = newEdge();

    FaceIter F_a_b_m = newFace();
    FaceIter F_b_d_m = newFace();

    // reassign elements
    // next, twin, vertex, edge, face
    m_c->setNeighbors(c_a, c_m, m, E_m_c, F_c_a_m);
    c_a->setNeighbors(a_m, a_c, c, E_c_a, F_c_a_m);
    a_m->setNeighbors(m_c, m_a, a, E_a_m, F_c_a_m);

    c_m->setNeighbors(m_d, m_c, c, E_m_c, F_c_m_d);
    m_d->setNeighbors(d_c, d_m, m, E_m_d, F_c_m_d);
    d_c->setNeighbors(c_m, c_d, d, E_d_c, F_c_m_d);

    b_m->setNeighbors(m_a, m_b, b, E_b_m, F_a_b_m);
    m_a->setNeighbors(a_b, a_m, m, E_a_m, F_a_b_m);
    a_b->setNeighbors(b_m, b_a, a, E_a_b, F_a_b_m);

    d_m->setNeighbors(m_b, m_d, d, E_m_d, F_b_d_m);
    m_b->setNeighbors(b_d, b_m, m, E_b_m, F_b_d_m);
    b_d->setNeighbors(d_m, d_b, b, E_b_d, F_b_d_m);

    a_c->setNeighbors(a_c->next(), c_a, a, E_c_a, a_c->face());
    c_d->setNeighbors(c_d->next(), d_c, c, E_d_c, c_d->face());
    d_b->setNeighbors(d_b->next(), b_d, d, E_b_d, d_b->face());
    b_a->setNeighbors(b_a->next(), a_b, b, E_a_b, b_a->face());

    m->position = (b->position + c->position) / 2.0;
    m->isNew = true;

    a->halfedge() = a_b;
    b->halfedge() = b_m;
    c->halfedge() = c_a;
    d->halfedge() = d_c;
    m->halfedge() = m_c;

    E_m_c->halfedge() = m_c;
    E_c_a->halfedge() = c_a;
    E_a_b->halfedge() = a_b;
    E_b_d->halfedge() = b_d;
    E_d_c->halfedge() = d_c;
    E_a_m->halfedge() = a_m;
    E_b_m->halfedge() = b_m;
    E_m_d->halfedge() = m_d;

    E_m_c->isNew = false;
    E_b_m->isNew = false;
    E_a_m->isNew = true;
    E_m_d->isNew = true;

    F_c_a_m->halfedge() = m_c;
    F_c_m_d->halfedge() = c_m;
    F_a_b_m->halfedge() = b_m;
    F_b_d_m->halfedge() = m_b;



    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      // set newPosition by weighted combination of surrounding vertices
      HalfedgeIter h = v->halfedge();
      Vector3D original_neighbor_position_sum(0, 0, 0);
      // loop all vertex around this v
      do {
        original_neighbor_position_sum += h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      int n = v->degree();
      float u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * n));
      v->newPosition = (1.0 - n * u) * v->position + u * original_neighbor_position_sum;

      v->isNew = false;
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      // inner triangle
      HalfedgeIter a_b = e->halfedge();
      HalfedgeIter b_d = a_b->next();
      HalfedgeIter d_a = b_d->next();
      HalfedgeIter b_a = a_b->twin();
      HalfedgeIter a_c = b_a->next();
      HalfedgeIter c_b = a_c->next();

      VertexIter a = a_b->vertex();
      VertexIter b = b_a->vertex();
      VertexIter c = c_b->vertex();
      VertexIter d = d_a->vertex();

      e->newPosition = 3.0 / 8.0 * (a->position + b->position) + 1.0 / 8.0 * (c->position + d->position);

      e->isNew = false;

    }
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v1 = e->halfedge()->vertex();
      VertexIter v2 = e->halfedge()->twin()->vertex();

      // only split old edges
      if (!v1->isNew && !v2->isNew) {
        VertexIter m = mesh.splitEdge(e);
        m->newPosition = e->newPosition;
      }
    }
    
    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v1 = e->halfedge()->vertex();
      VertexIter v2 = e->halfedge()->twin()->vertex();

      // flip new edges with old and new vertex
      if (e->isNew && (v1->isNew != v2->isNew)) {
        mesh.flipEdge(e);
      }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
      v->isNew = false;
    }
  }
}
