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
    std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const& points)
    {
        // TODO Part 1.

          // ��ǰ���Ƶ�
        std::vector<Vector2D> currentPoints = points;

        // de Casteljau �㷨
        int n = currentPoints.size();
        std::vector<Vector2D> newPoints;
        for (int i = 0; i < n - 1; i++) {
            newPoints.push_back(currentPoints[i] * (1 - t) + currentPoints[i + 1] * t);
        }
        return newPoints;
    }

    /**
     * Evaluates one step of the de Casteljau's algorithm using the given points and
     * the scalar parameter t (function parameter).
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return A vector containing intermediate points or the final interpolated vector
     */
    std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const& points, double t) const
    {

        std::vector<Vector3D> currentPoints = points;

        // de Casteljau �㷨
        int n = currentPoints.size();
        std::vector<Vector3D> newPoints;
        for (int i = 0; i < n - 1; i++) {
            newPoints.push_back(currentPoints[i] * (1 - t) + currentPoints[i + 1] * t);
        }
        return newPoints;
        // TODO Part 2.
        //return std::vector<Vector3D>();
    }

    /**
     * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
     *
     * @param points    A vector of points in 3D
     * @param t         Scalar interpolation parameter
     * @return Final interpolated vector
     */
    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const& points, double t) const
    {
        std::vector<Vector3D> currentPoints = points;
        while (currentPoints.size() > 1) {
            //if (currentPoints.size() == 1) break;
            currentPoints = evaluateStep(currentPoints, t);
        }
        return currentPoints[0];

        // TODO Part 2.
        //return Vector3D();
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
        std::vector<Vector3D> curvePoints;
        for (const auto& row : controlPoints) {
            curvePoints.push_back(evaluate1D(row, u)); // ����ÿһ�е� B��zier ���ߵ�
        }
        // ���� v �����µ� B��zier �����
        return evaluate1D(curvePoints, v);
        //return Vector3D();
    }

    Vector3D Vertex::normal(void) const
    {
        // TODO Part 3.
        // Returns an approximate unit normal at this vertex, computed by
        // taking the area-weighted average of the normals of neighboring
        // triangles, then normalizing.

        Vector3D normal(0.0, 0.0, 0.0);  // �����ۼӷ�����
        HalfedgeCIter h = halfedge();     // ��ȡ����İ��

        // ������ö������ڵ�����������
        HalfedgeCIter h_start = h;
        do {
            // ��ȡ��ǰ�����ε���������
            Vector3D p0 = h->vertex()->position;        // ��ǰ����
            Vector3D p1 = h->next()->vertex()->position;  // ��һ������
            Vector3D p2 = h->next()->next()->vertex()->position;  // ���¸�����

            // ���������η��ߣ�δ��һ����
            Vector3D face_normal = cross(p1 - p0, p2 - p0);

            // �������������
            double area = face_normal.norm() / 2.0;

            // ����Ȩ�����ۼӵ��ܷ���
            normal += area * face_normal;

            // �ƶ�����һ�������εİ��
            h = h->twin()->next();
        } while (h != h_start);

        // ����һ������
        return normal.unit();


        //return Vector3D();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
    {

        // TODO Part 4.
        // This method should flip the given edge and return an iterator to the flipped edge.
        HalfedgeIter h1 = e0->halfedge();
        HalfedgeIter h2 = h1->twin();

        if (h1->isBoundary() || h2->isBoundary()) {
            return e0;
        }
        VertexIter v1 = h1->vertex();
        VertexIter v2 = h2->vertex();
        VertexIter v3 = h1->next()->next()->vertex();
        VertexIter v4 = h2->next()->next()->vertex();

        FaceIter f1 = h1->face();
        FaceIter f2 = h2->face();

        HalfedgeIter h1_next = h1->next();
        HalfedgeIter h2_next = h2->next();
        HalfedgeIter h1_prev = h1_next->next();
        HalfedgeIter h2_prev = h2_next->next();

        /*HalfedgeIter h3 = h1_next->twin();
        HalfedgeIter h4 = h2_next->twin();
        HalfedgeIter h5 = h1_prev->twin();
        HalfedgeIter h6 = h2_prev->twin();*/

        h1->setNeighbors(h1_prev, h2, v4, e0, f1);
        h2->setNeighbors(h2_prev, h1, v3, e0, f2);

        h1_next->setNeighbors(h2, h1_next->twin(), v2, h1_next->edge(), f2);
        h2_next->setNeighbors(h1, h2_next->twin(), v1, h2_next->edge(), f1);

        h1_prev->setNeighbors(h2_next, h1_prev->twin(), v3, h1_prev->edge(), f1);
        h2_prev->setNeighbors(h1_next, h2_prev->twin(), v4, h2_prev->edge(), f2);


        f1->halfedge() = h1;
        f2->halfedge() = h2;


        return e0;
        //return EdgeIter();
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
    {
        // TODO Part 5.
        // This method should split the given edge and return an iterator to the newly inserted vertex.
        // The halfedge of this vertex should point along the edge that was split, rather than the new edges.



        // ��ȡ����������صİ�ߺͶ���
        HalfedgeIter h0 = e0->halfedge();
        HalfedgeIter h1 = h0->twin();

        VertexIter v0 = h0->vertex();
        VertexIter v1 = h1->vertex();

        // ����Ƿ��Ǳ߽��
        if (h0->isBoundary() || h1->isBoundary()) {
            return VertexIter();  // ����ڱ߽��ϣ�ֱ�ӷ��ؿյĵ�����
        }

        // ��ȡ���������ڵ��������
        HalfedgeIter h0_next = h0->next();
        HalfedgeIter h1_next = h1->next();
        HalfedgeIter h0_prev = h0_next->next();
        HalfedgeIter h1_prev = h1_next->next();

        // ��ȡ�ԽǶ���
        VertexIter v2 = h0_next->next()->vertex();
        VertexIter v3 = h1_next->next()->vertex();

        // �����¶����λ�ã��е�
        Vector3D new_position = (v0->position + v1->position) * 0.5;

        // �����µĶ��� m
        VertexIter m = newVertex();
        m->position = new_position;
        m->isNew = true;

        // �����µİ�ߡ��ߡ���
        HalfedgeIter H0 = h0;
        HalfedgeIter H1 = h1;
        HalfedgeIter h2 = newHalfedge();
        HalfedgeIter h3 = newHalfedge();
        HalfedgeIter h4 = newHalfedge();
        HalfedgeIter h5 = newHalfedge();
        HalfedgeIter h6 = newHalfedge();
        HalfedgeIter h7 = newHalfedge();

        EdgeIter E = H1->edge();
        EdgeIter e1 = newEdge();
        EdgeIter e2 = newEdge();
        EdgeIter e3 = newEdge();
       
        //e2->isNew = false;
        
        EdgeIter E1 = h0_next->edge();
        EdgeIter E2 = h1_next->edge();
        EdgeIter E3 = h0_prev->edge();
        EdgeIter E4 = h1_prev->edge();
        
        

        FaceIter f0 = h0->face();
        FaceIter f1 = h1->face();
        FaceIter f2 = newFace();
        FaceIter f3 = newFace();

        //Ϊ�����İ�����ù�ϵ
        h0->setNeighbors(h2, h1, v0, E, f0);
        h1->setNeighbors(h1_next, h0, m, E, f1);
        h2->setNeighbors(h0_prev, h3, m, e1, f0);
        h3->setNeighbors(h4, h2, v2, e1, f2);
        h4->setNeighbors(h0_next, h5, m, e2, f2);
        h5->setNeighbors(h6, h4, v1, e2, f3);
        h6->setNeighbors(h1_prev, h7, m, e3, f3);
        h7->setNeighbors(h1, h6, v3, e3, f1);
        
        h0_next->setNeighbors(h3, h0_next->twin(), v1, h0_next->edge(), f2);
        h0_prev->setNeighbors(h0, h0_prev->twin(), v2, h0_prev->edge(), f0);
        h1_next->setNeighbors(h7, h1_next->twin(), v0, h1_next->edge(), f1);
        h1_prev->setNeighbors(h5, h1_prev->twin(), v3, h1_prev->edge(), f3);

        v0->halfedge() = H0;
        v1->halfedge() = h5;
        v2->halfedge() = h3;
        v3->halfedge() = h7;
        m->halfedge() = h1;

        E->halfedge() = H0;
        e1->halfedge() = h2;
        e1->isNew = true;
        e2->halfedge() = h4;
        e3->halfedge() = h6;
        e3->isNew = true;

        E1->halfedge() = h0_next;
        E2->halfedge() = h1_next;
        E3->halfedge() = h0_prev;
        E4->halfedge() = h1_prev;

        f0->halfedge() = H0;
        f1->halfedge() = h1;
        f2->halfedge() = h4;
        f3->halfedge() = h5;

        return m;
    }



    void MeshResampler::upsample(HalfedgeMesh& mesh)
    {
        // TODO Part 6.
        // This routine should increase the number of triangles in the mesh using Loop subdivision.
        // One possible solution is to break up the method as listed below.

        // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // a vertex of the original mesh.

        // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.

        // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
        // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
        // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
        // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

        // 4. Flip any new edge that connects an old and new vertex.

        // 5. Copy the new vertex positions into final Vertex::position.
        
    // ���� 1: �������ж���ͱߵ� isNew ��־
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
            v->isNew = false;
        }
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
            e->isNew = false;
        }

        // ���� 2: ����ߵ���λ��
        std::vector<EdgeIter> edgesToSplit;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
            HalfedgeIter h = e->halfedge();
            Vector3D A = h->vertex()->position;
            Vector3D B = h->twin()->vertex()->position;
            Vector3D C = h->next()->next()->vertex()->position;
            Vector3D D = h->twin()->next()->next()->vertex()->position;

            // ʹ�� Loop ϸ�ֹ������ߵ���λ��
            e->newPosition = (3.0f / 8.0f) * (A + B) + (1.0f / 8.0f) * (C + D);
            edgesToSplit.push_back(e);
        }

        // ���� 3: �������бߣ���Ϊ�¶�������µ�λ��
        for (EdgeIter e : edgesToSplit) {
            VertexIter newVertex = mesh.splitEdge(e);
            newVertex->position = e->newPosition;
        }

        // ���� 4: ��ת���Ӿɶ�����¶���ı�
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
            if (e->isNew) {
                VertexIter v1 = e->halfedge()->vertex();
                VertexIter v2 = e->halfedge()->twin()->vertex();

                // �����������һ���ɶ����һ���¶��㣬��ת�ñ�
                if ((v1->isNew && !v2->isNew) || (!v1->isNew && v2->isNew)) {
                    mesh.flipEdge(e);
                }
            }
        }

        // ���� 5: ʹ�� Loop ϸ�ֹ������ԭ�������λ��
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
            if (!v->isNew) {
                size_t n = v->degree();  // ��ȡ����Ķ���
                double u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * n));  // ���ڶ�����Ȩ������
                Vector3D neighborSum(0, 0, 0);

                // �������ڶ���λ�õĺ�
                HalfedgeIter h = v->halfedge();
                HalfedgeIter hStart = h;
                do {
                    neighborSum += h->twin()->vertex()->position;
                    h = h->twin()->next();
                } while (h != hStart);

                // ���¶������λ��
                v->newPosition = (1.0 - n * u) * v->position + u * neighborSum;
            }
        }

        // ���� 6: ����λ�÷��������ԭ����
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
            if (!v->isNew) {
                v->position = v->newPosition;
            }
        }

        // ���� 7: �Ծɶ����һ��ƽ��λ��
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
            if (!v->isNew) {
                Vector3D neighborSum(0, 0, 0);
                HalfedgeIter h = v->halfedge();

                // �������ڶ���λ�õĺ�
                do {
                    neighborSum += h->twin()->vertex()->position;
                    h = h->twin()->next();
                } while (h != v->halfedge());

                // ʹ�� Loop ϸ��Ȩ�����¼��㶥��λ��
                size_t n = v->degree();
                double u = (n == 3) ? (3.0 / 16.0) : (3.0 / (8.0 * n));
                v->position = (1.0 - n * u) * v->position + u * neighborSum;
            }
        }
    }
}
