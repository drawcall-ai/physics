#include <coacd.h>
#include <emscripten/bind.h>

using emscripten::val;

val decompose(val vertices, val indices) {
  coacd::Mesh mesh;
  const auto vertexCount = vertices["length"].as<unsigned>() / 3;
  const auto faceCount = indices["length"].as<unsigned>() / 3;
  for (unsigned i = 0; i < vertexCount; ++i)
    mesh.vertices.push_back({vertices[i * 3].as<double>(),
                             vertices[i * 3 + 1].as<double>(),
                             vertices[i * 3 + 2].as<double>()});
  for (unsigned i = 0; i < faceCount; ++i)
    mesh.indices.push_back({indices[i * 3].as<int>(),
                           indices[i * 3 + 1].as<int>(),
                           indices[i * 3 + 2].as<int>()});
  coacd::set_log_level("error");
  auto parts = coacd::CoACD(mesh, 0.05, -1, "off");
  auto hulls = val::array();
  for (const auto &part : parts) {
    auto points = val::array();
    for (const auto &point : part.vertices)
      for (double coordinate : point) points.call<void>("push", coordinate);
    hulls.call<void>("push", points);
  }
  return hulls;
}

EMSCRIPTEN_BINDINGS(coacd) {
  emscripten::function("decompose", &decompose);
}
