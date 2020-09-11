#include "meshcp_query.h"
#include <stdarg.h>

box3::box3() : min(UNSET), max(-UNSET) { }

void box3::inflate_one(const glm::vec3& pt)
{
    min.x = std::min(pt.x, min.x);
    min.y = std::min(pt.y, min.y);
    min.z = std::min(pt.z, min.z);
    max.x = std::max(pt.x, max.x);
    max.y = std::max(pt.y, max.y);
    max.z = std::max(pt.z, max.z);
}

template<>
inline void box3::inflate(const glm::vec3& pt)
{
    inflate_one(pt);
}

meshcp_query_base::meshcp_query_base(const mesh& m) : m_mesh(m)
{
}

void mesh::compute_normals()
{
    facenormals.clear();
    facenormals.reserve(faces.size());
    for (const mesh::face_type& face : faces)
    {
        const glm::vec3& a = vertices.at(face[0]);
        const glm::vec3& b = vertices.at(face[1]);
        const glm::vec3& c = vertices.at(face[2]);
        facenormals.push_back(glm::normalize(glm::cross(b - a, c - a)));
    }
}

mesh::mesh(const glm::vec3* verts, uint32_t nVerts, const uint32_t* triIndices, uint32_t nTriangles) :
    vertices(verts, verts + nVerts), faces(nTriangles)
{
    std::memcpy(faces.data(), triIndices, sizeof(uint32_t) * 3 * nTriangles);
    compute_normals();
}

void mesh::populate_facetree(boost_rtree& tree)
{
    std::vector<rtree_item> items;
    items.reserve(faces.size());
    for (uint32_t fi = 0; fi < faces.size(); fi++)
    {
        const mesh::face_type& face = faces.at(fi);
        box3 box;
        box.inflate(vertices[face[0]], vertices[face[1]], vertices[face[2]]);
        items.emplace_back(box.boostbox, fi);
    }
    tree.insert(items.cbegin(), items.cend());
}

glm::vec3 mesh::face_closest_pt(uint32_t faceIndex, const glm::vec3& pt) const
{
    const face_type& face = faces.at(faceIndex);
    const glm::vec3& facenorm = facenormals.at(faceIndex);
    const glm::vec3& tri0 = vertices.at(face[0]);
    const glm::vec3& tri1 = vertices.at(face[1]);
    const glm::vec3& tri2 = vertices.at(face[2]);
    glm::vec3 projected = (facenorm * glm::dot(tri0 - pt, facenorm)) + pt;

    glm::vec3 edge0 = tri1 - tri0;
    glm::vec3 edge1 = tri2 - tri0;
    glm::vec3 v0 = tri0 - projected;

    float dot00 = glm::dot(edge0, edge0);
    float dot01 = glm::dot(edge0, edge1);
    float dot11 = glm::dot(edge1, edge1);
    float dot0v = glm::dot(edge0, v0);
    float dot1v = glm::dot(edge1, v0);

    float det = dot00 * dot11 - dot01 * dot01;
    float s = dot01 * dot1v - dot11 * dot0v;
    float t = dot01 * dot0v - dot00 * dot1v;

    if (s + t < det)
    {
        if (s < 0.f)
        {
            if (t < 0.f)
            {
                if (dot0v < 0.f)
                {
                    s = clamp(-dot0v / dot00, 0.f, 1.f);
                    t = 0.f;
                }
                else
                {
                    s = 0.f;
                    t = clamp(-dot1v / dot11, 0.f, 1.f);
                }
            }
            else
            {
                s = 0.f;
                t = clamp(-dot1v / dot11, 0.f, 1.f);
            }
        }
        else if (t < 0.f)
        {
            s = clamp(-dot0v / dot00, 0.f, 1.f);
            t = 0.f;
        }
        else
        {
            float invDet = 1.0f / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else
    {
        if (s < 0.f)
        {
            float tmp0 = dot01 + dot0v;
            float tmp1 = dot11 + dot1v;
            if (tmp1 > tmp0)
            {
                float numer = tmp1 - tmp0;
                float denom = dot00 - 2 * dot01 + dot11;
                s = clamp(numer / denom, 0.f, 1.f);
                t = 1 - s;
            }
            else
            {
                t = clamp(-dot1v / dot11, 0.f, 1.f);
                s = 0.f;
            }
        }
        else if (t < 0.f)
        {
            if (dot00 + dot0v > dot01 + dot1v)
            {
                float numer = dot11 + dot1v - dot01 - dot0v;
                float denom = dot00 - 2 * dot01 + dot11;
                s = clamp(numer / denom, 0.f, 1.f);
                t = 1 - s;
            }
            else
            {
                s = clamp(-dot1v / dot11, 0.f, 1.f);
                t = 0.f;
            }
        }
        else
        {
            float numer = dot11 + dot1v - dot01 - dot0v;
            float denom = dot00 - 2 * dot01 + dot11;
            s = clamp(numer / denom, 0.f, 1.f);
            t = 1.f - s;
        }
    }

    return tri0 + s * edge0 + t * edge1;
}
