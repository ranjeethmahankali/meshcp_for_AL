#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <vector>
#include <glm/glm.hpp>
#include <cfloat>
#include <bitset>
#include <thread>
#include <chrono>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
namespace bgm = boost::geometry::model;
constexpr unsigned int RTREE_NUM_ELEMENTS_PER_NODE = 16;

using boost_vec3f = bgm::point<float, 3, bg::cs::cartesian>;
using boost_box3f = bgm::box<boost_vec3f>;
using rtree_item = std::pair<boost_box3f, uint32_t>;
using boost_rtree = bgi::rtree<rtree_item, bgi::quadratic<RTREE_NUM_ELEMENTS_PER_NODE>>;

#define UNSET { FLT_MAX, FLT_MAX, FLT_MAX }

template <typename T>
float clamp(T val, T min, T max)
{
    return
        val < min ? min :
        val > max ? max :
        val;
};

float squared_length(const glm::vec3& a);

struct edge_info
{
    glm::vec3 start;
    glm::vec3 vector;
    glm::vec3 l2_scaled = UNSET;

    edge_info() = default;
    edge_info(const glm::vec3& v1, const glm::vec3& v2);

    void closest_point(const glm::vec3& pt, const glm::vec3& projected, float& squaredDistance, glm::vec3& dest) const;
    float orientation(const glm::vec3& pt, const glm::vec3& facenormal) const;
};

struct mesh
{
private:
    void compute_normals();
    void cache_edge_info();
public:
    using face_type = std::array<uint32_t, 3>;
    using edgeset_type = std::array<edge_info, 3>;
    std::vector<glm::vec3> vertices;
    std::vector<face_type> faces;
    std::vector<glm::vec3> face_normals;
    std::vector<edgeset_type> face_edges;

    mesh() = default;
    mesh(const glm::vec3* verts, uint32_t nVerts, const uint32_t* triIndices, uint32_t nTriangles);

    void populate_facetree(boost_rtree& tree);
    void populate_vertextree(boost_rtree& tree);
    void face_closest_pt(uint32_t faceIndex, const glm::vec3& pt, float& squaredDistance, glm::vec3& dest) const;
};

struct box3
{
    /*IMPORTANT: This union only works if the data layout of the boost rtree is {xmin, ymin, zmin, xmax, ymax, zmax}.
    If, for some reason, this is changed, this union will not work.*/
    static_assert(
        sizeof(glm::vec3) == sizeof(boost_vec3f) &&
        sizeof(boost_box3f) == sizeof(glm::vec3) * 2,
        "The use of union in this structure doesn't make sense.");
public:
    struct simple_box { glm::vec3 min, max; };
    union
    {
        simple_box simplebox;
        boost_box3f boostbox;
    };

    box3();

    void inflate(const glm::vec3& pt);
    void inflate(float dist);

    template <typename T, typename... TMore>
    void inflate(T first, const TMore&... more)
    {
        inflate(first);
        inflate(more...);
    }
};

namespace options
{
    using options_type = uint32_t;
    constexpr options_type CONCURRENCY =    0x0000000f;
    constexpr options_type SERIAL =         0x00000001;
    constexpr options_type PARALLEL =       0x00000002;
}

class meshcp_query
{
    mesh m_mesh;
    boost_rtree m_facetree, m_verttree;
    mutable std::vector<rtree_item> serialResultsBuf;

    template <options::options_type Options>
    glm::vec3 search_query_box(const glm::vec3& pt, const box3& queryBox) const {};

public:
    meshcp_query(const mesh& m) : m_mesh(m)
    {
        m_mesh.populate_facetree(m_facetree);
        m_mesh.populate_vertextree(m_verttree);
        serialResultsBuf.reserve(m_mesh.faces.size());
    };

    template <options::options_type Options>
    glm::vec3 run(const glm::vec3& pt, float maxDistance) const
    {
        box3 box;
        rtree_item nearestVert(box.boostbox, UINT32_MAX);
        m_verttree.query(bgi::nearest(boost_vec3f(pt.x, pt.y, pt.z), 1), &nearestVert);
        if (nearestVert.second == UINT32_MAX) return UNSET;
        float vertDistance = glm::distance(m_mesh.vertices.at(nearestVert.second), pt);
        if (vertDistance > maxDistance) return UNSET;

        maxDistance = vertDistance * 1.01f;

        box.inflate(pt);
        box.inflate(maxDistance);
        return search_query_box<Options & options::CONCURRENCY>(pt, box);
    };

    glm::vec3 operator()(const glm::vec3& pt, float maxDistance) const;
};

template <>
inline glm::vec3 meshcp_query::search_query_box<options::SERIAL>(const glm::vec3& pt, const box3& queryBox) const
{
    glm::vec3 best = UNSET;
    float bestDistSq = FLT_MAX;
    serialResultsBuf.clear();
    m_facetree.query(bgi::intersects(queryBox.boostbox), std::back_inserter(serialResultsBuf));
    for (const rtree_item& item : serialResultsBuf)
        m_mesh.face_closest_pt(item.second, pt, bestDistSq, best);
    return best;
}

template <>
inline glm::vec3 meshcp_query::search_query_box<options::PARALLEL>(const glm::vec3& pt, const box3& queryBox) const
{
    glm::vec3 best = UNSET;
    float bestDistSq = FLT_MAX;
    std::vector<rtree_item> queryResults;
    m_facetree.query(bgi::intersects(queryBox.boostbox), std::back_inserter(queryResults));
    for (const rtree_item& item : queryResults)
        m_mesh.face_closest_pt(item.second, pt, bestDistSq, best);
    return best;
}

template <size_t NThreads>
class parallel_meshcp_query : public meshcp_query
{
    std::array<std::thread, NThreads> m_threads;
    uint64_t m_timeMicroSeconds = 0;
    const glm::vec3* const m_points;
    glm::vec3* const m_results;
    size_t m_numPoints;
    float m_maxDistance;

    void run_range(size_t i, size_t j)
    {
        while (i != j)
        {
            m_results[i] = run<options::PARALLEL>(m_points[i], m_maxDistance);
            if (++i > m_numPoints) break;
        }
    }

public:
    parallel_meshcp_query(const mesh& m, const glm::vec3* const points, glm::vec3* const results, size_t numPoints, float maxDistance) :
        meshcp_query(m), m_points(points), m_results(results), m_numPoints(numPoints), m_maxDistance(maxDistance)
    {
    };

    void run_parallel()
    {
        size_t rangeSize = m_numPoints / NThreads;
        auto timeStart = std::chrono::high_resolution_clock::now();
        for (size_t i = 0; i < NThreads; i++)
        {
            size_t start = rangeSize * i;
            size_t end = i == NThreads - 1 ? m_numPoints : start + rangeSize;
            m_threads[i] = std::thread(&parallel_meshcp_query::run_range, this, start, end);
        }

        for (std::thread& t : m_threads)
            t.join();
        auto timeEnd = std::chrono::high_resolution_clock::now();
        m_timeMicroSeconds = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart).count();
    };

    uint64_t time_taken() const
    {
        return m_timeMicroSeconds;
    }
};

template <size_t Depth>
struct octree_bits
{
    static constexpr size_t NUM_BITS = 1 << (3 * Depth);
    static constexpr uint8_t NUM_CHILDREN = 8;
    using child_type = octree_bits<Depth - 1>;
    union
    {
        std::bitset<NUM_BITS> m_bits;
        child_type m_children[NUM_CHILDREN];
    };

    constexpr size_t depth() const noexcept { return Depth; };
    bool full() const noexcept { return m_bits.all(); };
    bool empty() const noexcept { return m_bits.none(); }
    bool num_active() const noexcept { return m_bits.count(); };
    
    template <uint8_t ChildIndex>
    const child_type& child() const noexcept
    {
        static_assert(ChildIndex < NUM_CHILDREN, "An Octree has at most 8 children.");
        return m_bits[ChildIndex];
    };
};

template <>
struct octree_bits<1>
{
    static constexpr size_t NUM_BITS = 8;
    static constexpr uint8_t NONE = 0;
    static constexpr uint8_t ALL = ~(0);
    uint8_t m_bits;

    constexpr size_t depth() const noexcept { return 1; };
    bool full() const noexcept { return m_bits == ALL; };
    bool empty() const noexcept { return m_bits == NONE; }
    //bool num_active() const noexcept { return ; };
};