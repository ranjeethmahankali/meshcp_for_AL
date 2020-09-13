#include "meshcp_query.h"
#include <stdarg.h>

box3::box3() : min(UNSET), max(-UNSET) { }

void box3::inflate(const glm::vec3& pt)
{
    min.x = std::min(pt.x, min.x);
    min.y = std::min(pt.y, min.y);
    min.z = std::min(pt.z, min.z);
    max.x = std::max(pt.x, max.x);
    max.y = std::max(pt.y, max.y);
    max.z = std::max(pt.z, max.z);
}

void box3::inflate(float dist)
{
    dist = std::max(0.0f, dist);
    glm::vec3 half(dist, dist, dist);
    min -= half;
    max += half;
}

void mesh::compute_normals()
{
    face_normals.clear();
    face_normals.reserve(faces.size());
    for (const mesh::face_type& face : faces)
    {
        const glm::vec3& a = vertices.at(face[0]);
        const glm::vec3& b = vertices.at(face[1]);
        const glm::vec3& c = vertices.at(face[2]);
        face_normals.push_back(glm::normalize(glm::cross(b - a, c - a)));
    }
}

void mesh::cache_edge_info()
{
    face_edges.clear();
    face_edges.reserve(faces.size());
    for (const mesh::face_type& face : faces)
    {
        face_edges.push_back({
            edge_info(vertices[face[1]], vertices[face[2]]),
            edge_info(vertices[face[2]], vertices[face[0]]),
            edge_info(vertices[face[0]], vertices[face[1]]),
        });
    }
}

mesh::mesh(const glm::vec3* verts, uint32_t nVerts, const uint32_t* triIndices, uint32_t nTriangles) :
    vertices(verts, verts + nVerts), faces(nTriangles)
{
    std::memcpy(faces.data(), triIndices, sizeof(uint32_t) * 3 * nTriangles);
    compute_normals();
    cache_edge_info();
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

void mesh::populate_vertextree(boost_rtree& tree)
{
    std::vector<rtree_item> items;
    items.reserve(faces.size());
    for (uint32_t vi = 0; vi < vertices.size(); vi++)
    {
        box3 box;
        box.inflate(vertices.at(vi));
        items.emplace_back(box.boostbox, vi);
    }
    tree.insert(items.cbegin(), items.cend());
}

void mesh::face_closest_pt(uint32_t faceIndex, const glm::vec3& pt, float& squaredDistance, glm::vec3& dest) const
{
    const glm::vec3& facenorm = face_normals.at(faceIndex);
    glm::vec3 projection = facenorm * glm::dot(vertices.at(faces.at(faceIndex)[0]) - pt, facenorm);
    float projLenSq = squared_length(projection);
    if (projLenSq >= squaredDistance)
        return;
    glm::vec3 projected = projection + pt;

    const edgeset_type& edges = face_edges.at(faceIndex);
    uint8_t nOutside = 0;
    for (const edge_info& edge : edges)
    {
        if (edge.orientation(pt, facenorm) < 0.0f)
        {
            nOutside++;
            edge.closest_point(pt, projected, squaredDistance, dest);
        }
        if (nOutside > 1)
            break;
    }

    if (nOutside == 0)
    {
        squaredDistance = projLenSq;
        dest = projected;
    }
}

glm::vec3 meshcp_query::operator()(const glm::vec3& pt, float maxDistance) const
{
    return run<options::SERIAL>(pt, maxDistance);
}

float squared_length(const glm::vec3& a)
{
    return a.x * a.x + a.y * a.y + a.z * a.z;
}

edge_info::edge_info(const glm::vec3& v1, const glm::vec3& v2) :
    start(v1), vector(v2 - v1)
{
    l2_scaled = vector / glm::dot(vector, vector);
}

void edge_info::closest_point(const glm::vec3& pt, const glm::vec3& projected, float& squaredDistance, glm::vec3& dest) const
{
    glm::vec3 temp = start + clamp(glm::dot(l2_scaled, projected - start), 0.0f, 1.0f) * vector;
    float distSqTemp = squared_length(temp - pt);
    if (distSqTemp < squaredDistance)
    {
        dest = temp;
        squaredDistance = distSqTemp;
    }
}

float edge_info::orientation(const glm::vec3& pt, const glm::vec3& facenormal) const
{
    return glm::dot(glm::cross(vector, pt - start), facenormal);
}
