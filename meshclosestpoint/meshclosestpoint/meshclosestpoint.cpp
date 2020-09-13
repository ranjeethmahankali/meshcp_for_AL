#include <iostream>
#include <type_traits>
#include <glm/glm.hpp>
#include <meshcp_query.h>
#include <fstream>
#include <chrono>
#include <execution>

#define SERIAL_TYPE_ERR_MSG "This datatype cannot be serialized / deserialzed."

template <typename T>
struct is_serial : public std::is_fundamental<T> {};

template <>
struct is_serial<glm::vec3> : public std::true_type {};

template <typename T>
size_t deserialize(char* src, size_t pos, T& dest)
{
    static_assert(is_serial<T>::value, SERIAL_TYPE_ERR_MSG);
    std::memcpy(&dest, src + pos, sizeof(T));
    return pos + sizeof(T);
};

template <typename T>
size_t deserialize(char* src, size_t pos, T* dest, size_t numElements)
{
    static_assert(is_serial<T>::value, SERIAL_TYPE_ERR_MSG);
    size_t nBytes = numElements * sizeof(T);
    std::memcpy(dest, src + pos, nBytes);
    return pos + nBytes;
};

template <typename T>
size_t serialize(char* dest, size_t pos, const T& data)
{
    static_assert(is_serial<T>::value, SERIAL_TYPE_ERR_MSG);
    std::memcpy(dest + pos, &data, sizeof(T));
    return pos + sizeof(T);
};

template <typename T>
size_t serialize(char* dest, size_t pos, const T* const data, size_t numElements)
{
    static_assert(is_serial<T>::value, SERIAL_TYPE_ERR_MSG);
    size_t nBytes = numElements * sizeof(T);
    std::memcpy(dest + pos, data, nBytes);
    return pos + nBytes;
}

bool read_bytes(const std::string& filepath, std::vector<char>& bytes)
{
    std::ifstream ifs(filepath, std::ios::binary | std::ios::ate); // Seek to the end of the file with ate.
    if (!ifs.is_open())
    {
        std::cerr << "Unable to open the file - " << filepath << std::endl;
        return false;
    }
    std::ifstream::pos_type length = ifs.tellg();
    bytes.clear();
    bytes.resize(length);
    ifs.seekg(0, std::ios::beg);
    ifs.read(bytes.data(), length);
    ifs.close();
    return true;
}

mesh read_mesh(const std::string& filepath)
{
    std::vector<char> bytes;
    if (!read_bytes(filepath, bytes)) return mesh();
    uint32_t nVerts = 0;
    uint32_t nFaces = 0;
    size_t pos = 0;
    char* src = bytes.data();
    pos = deserialize(src, pos, nVerts);
    pos = deserialize(src, pos, nFaces);
    size_t offset = sizeof(nVerts) + sizeof(nFaces);
    return mesh(
        (glm::vec3*)(src + offset), nVerts,
        (const uint32_t*)(src + offset + nVerts * sizeof(glm::vec3)), nFaces);
}

void read_points(const std::string& filepath, std::vector<glm::vec3>& samples, std::vector<glm::vec3>& results)
{
    std::vector<char> bytes;
    if (!read_bytes(filepath, bytes)) return;
    samples.clear();
    results.clear();
    uint32_t nPoints = 0;
    size_t pos = 0;
    char* src = bytes.data();
    pos = deserialize(src, pos, nPoints);

    samples.resize(nPoints);
    results.resize(nPoints);
    pos = deserialize(src, pos, samples.data(), nPoints);
    pos = deserialize(src, pos, results.data(), nPoints);
    assert(pos == bytes.size()); // EOF
}

bool write_bytes(const std::string& filepath, const char* const bytes, size_t nBytes)
{
    std::ofstream ofs(filepath, std::ios::binary | std::ios::out);
    if (!ofs.is_open())
    {
        std::cerr << "Unable to write file - " << filepath << std::endl;
        return false;
    }
    ofs.write(bytes, nBytes);
    ofs.close();
    return true;
}

void write_results(const std::string& filepath, const std::vector<glm::vec3>& results)
{
    std::vector<char> bytes(sizeof(uint32_t) + results.size() * sizeof(glm::vec3));
    size_t pos = 0;
    char* dest = bytes.data();
    pos = serialize(dest, pos, (uint32_t)results.size());
    pos = serialize(dest, pos, results.data(), results.size());
    assert(pos == bytes.size()); // EOF

    write_bytes(filepath, bytes.data(), bytes.size());
};

float max_error(const std::vector<glm::vec3>& expected, const std::vector<glm::vec3>& results, const std::vector<glm::vec3>& points)
{
    size_t nPoints = std::min(expected.size(), results.size());
    float errmax = 0.0f;
    for (size_t i = 0; i < expected.size(); i++)
    {
        float error = glm::distance(results[i], points[i]) - glm::distance(expected[i], points[i]);
        errmax = std::max(errmax, error);
    }
    return errmax;
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Please supply the name of the test case.\n";
        return 1;
    }
    std::string name(argv[1]);
    std::cout << "=============================\n";
    std::cout << "Test case: " << name << std::endl;
    std::cout << "=============================\n\n";
    std::string meshfile = name + "_mesh.dat";
    std::cout << "Reading mesh from " << meshfile << std::endl;
    mesh msh = read_mesh(meshfile);
    std::vector<glm::vec3> points, expected;
    std::string pointsfile = name + "_points.dat";
    std::cout << "Reading points from " << meshfile << std::endl;
    read_points(pointsfile, points, expected);

    meshcp_query query(msh);
    std::vector<glm::vec3> results;
    results.reserve(points.size());


    std::cout << "\nPerforming serial MeshCP query with " << points.size() << " test points\n";
    auto start = std::chrono::high_resolution_clock::now();
    for (const glm::vec3& pt : points)
    {
        results.push_back(query(pt, 100.0f));
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds\n";

    float error = max_error(expected, results, points);
    std::cout << "Maximum deviation from the expected results: " << error << std::endl;

    std::vector<uint32_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), 0);
    results.resize(indices.size());
    results.clear();
    results.resize(indices.size());
    std::cout << "\nPerforming parallel MeshCP query with " << points.size() << " test points\n";
    start = std::chrono::high_resolution_clock::now();
    std::for_each(std::execution::par_unseq, indices.cbegin(), indices.cend(),
        [&points, &query, &results](uint32_t i) {
            results[i] = query.run<options::PARALLEL>(points[i], 100.0f);
        });
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds\n";
    error = max_error(expected, results, points);
    std::cout << "Maximum deviation from the expected results: " << error << std::endl;

    std::string resultsfile = name + "_results.dat";
    std::cout << "\nWriting results to - " << resultsfile << std::endl;
    write_results(resultsfile, results);
    std::cout << "Done.\n";
    return 0;
}