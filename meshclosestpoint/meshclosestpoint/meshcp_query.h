#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <vector>
#include <glm/glm.hpp>
#include <cfloat>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgm = boost::geometry::model;
constexpr unsigned int RTREE_NUM_ELEMENTS_PER_NODE = 16;

using boost_vec3f = bgm::point<float, 3, bg::cs::cartesian>;
using boost_box3f = bgm::box<boost_vec3f>;
using rtree_item = std::pair<boost_box3f, uint32_t>;
using boost_rtree = bgi::rtree<rtree_item, bgi::quadratic<RTREE_NUM_ELEMENTS_PER_NODE>>;

constexpr glm::vec3 UNSET{ FLT_MAX, FLT_MAX, FLT_MAX };

template <typename T>
float clamp(T val, T min, T max)
{
    return
        val < min ? min :
        val > max ? max :
        val;
};

struct mesh
{
private:
    void compute_normals();
public:
    using face_type = std::array<uint32_t, 3>;
    std::vector<glm::vec3> vertices;
    std::vector<face_type> faces;
    std::vector<glm::vec3> facenormals;

    mesh(const glm::vec3* verts, uint32_t nVerts, const uint32_t* triIndices, uint32_t nTriangles);

    void populate_facetree(boost_rtree& tree);
    glm::vec3 face_closest_pt(uint32_t faceIndex, const glm::vec3& pt) const;
};

union box3
{
    struct
    {
        glm::vec3 min;
        glm::vec3 max;
    };
    boost_box3f boostbox;

    box3();

    void inflate_one(const glm::vec3& pt);

    template <typename... T>
    void inflate(const T&... morePts) {};

    template <typename... T>
    void inflate(const glm::vec3& pt, const T&... morePts)
    {
        inflate_one(pt);
        inflate<T...>(morePts...);
    }

    template<>
    void inflate<glm::vec3>(const glm::vec3& pt);
};

namespace strategies
{
    constexpr uint8_t NONE = 0;
    constexpr uint8_t VERT_FILTER = 1;
    constexpr uint8_t PARALLEL = 2;
    constexpr uint8_t SPHERE_BFS = 3;
}

class meshcp_query_base
{
protected:
    mesh m_mesh;
    meshcp_query_base(const mesh&);
};

template <uint8_t Strategy>
class meshcp_query
{
    glm::vec3 operator()(const glm::vec3& pt, float maxDistance) const
    {
        static_assert(false, "Template specializations needed to implement specific strategies");
    }
};

template <>
class meshcp_query<strategies::NONE> : public meshcp_query_base
{
    boost_rtree m_facetree;
    mutable std::vector<rtree_item> m_queryResults;

public:
    meshcp_query(const mesh& m) : meshcp_query_base(m)
    {
        m_mesh.populate_facetree(m_facetree);
        m_queryResults.reserve(m.faces.size());
    };

    glm::vec3 operator()(const glm::vec3& pt, float maxDistance) const
    {
        box3 box;
        maxDistance = std::abs(maxDistance);
        glm::vec3 halfVec(maxDistance, maxDistance, maxDistance);
        box.min = pt - halfVec;
        box.max = pt + halfVec;
        m_queryResults.clear();
        m_facetree.query(bgi::intersects(box.boostbox), std::back_inserter(m_queryResults));

        glm::vec3 best = UNSET;
        float bestDist = FLT_MAX;
        for (const rtree_item& item : m_queryResults)
        {
            glm::vec3 temp = m_mesh.face_closest_pt(item.second, pt);
            float dist = glm::distance(temp, pt);
            if (dist < bestDist)
            {
                best = temp;
                bestDist = dist;
            }
        }

        return best;
    };
};

template <>
class meshcp_query<strategies::VERT_FILTER | strategies::PARALLEL>;