/**
 * @File: test_main.cpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Test main function that reads in a simple graph.
 **/

/* C++ Headers */
#include<random>
#include<iomanip>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<street_graph/graph.hpp>

/* Plotting Headers */
#include<matplotlibcpp/matplotlibcpp.hpp>

inline static constexpr const double                  start_time          = 0;
inline static constexpr const double                  end_time            = 48*60*60;
inline static constexpr const double                  shift_length        = 12.0 * 60.0 * 60.0;
inline static constexpr const size_t                  num_agent_active    = 4;
inline static constexpr const Eigen::Index            NUM_AGENTS          = std::ceil((end_time - start_time) / (shift_length / double(num_agent_active))) + (int64_t(num_agent_active) - 1);
inline static constexpr const Eigen::Index            NUM_HOTSPOTS        = 75; // 25 50;
inline static constexpr const Eigen::Index            NUM_EVENT_POINTS    = 30; // 10 30;
inline static constexpr const Eigen::StorageOptions   EIG_OPTIONS         = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign);

std::vector<double> toVec(const Eigen::Ref<const Eigen::Matrix<float,Eigen::Dynamic,1>>& input)
{
  std::vector<double> output(input.rows());

  for(Eigen::Index col_it = 0; col_it < input.rows(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}
std::vector<double> toVec(const Eigen::Ref<const Eigen::Matrix<double,Eigen::Dynamic,1>>& input)
{
  std::vector<double> output(input.rows());

  for(Eigen::Index col_it = 0; col_it < input.rows(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}

int main()
{
  const std::string base_dir("/home/james/ros_ws/src/graph_surveillance_planning/street_graph/config/");
//  graph::Graph      graph(base_dir+"los_angeles_3500_nodes.csv", base_dir+"los_angeles_3500_edges.csv", -1, false);
  graph::Graph      graph(base_dir+"new_york_15000_nodes.csv", base_dir+"new_york_15000_edges.csv", -1, false);
//  graph::Graph      graph(base_dir+"seattle_5500_nodes.csv", base_dir+"seattle_5500_edges.csv", -1, false);
//  graph.edgesToFile("/tmp/los_angeles_2500", 1);

  std::cout << "Original Node Count:   " << graph.numberOriginalNodes()   << std::endl;
  std::cout << "Simplified Node Count: " << graph.numberSimplifiedNodes() << std::endl;
  std::cout << "Original Edge Count:   " << graph.numberOriginalEdges()   << std::endl;
  std::cout << "Simplified Edge Count: " << graph.numberSimplifiedEdges() << std::endl;
  std::cout << "Number of Agents:      " << NUM_AGENTS                    << std::endl;

  const boost::integer_range<uint32_t> orig_node_inds(0, graph.numberOriginalNodes());
  std::cout << "Max x:                 " << graph.cgetOriginalNodes()[*std::max_element(orig_node_inds.begin(), orig_node_inds.end(),
                                                                      [&] (const size_t i, const size_t j) -> bool { return graph.cgetOriginalNodes()[i]->cgetPosition()[0] < graph.cgetOriginalNodes()[j]->cgetPosition()[0]; })]->cgetPosition()[0] << std::endl;
  std::cout << "Min x:                 " << graph.cgetOriginalNodes()[*std::min_element(orig_node_inds.begin(), orig_node_inds.end(),
                                                                      [&] (const size_t i, const size_t j) -> bool { return graph.cgetOriginalNodes()[i]->cgetPosition()[0] < graph.cgetOriginalNodes()[j]->cgetPosition()[0]; })]->cgetPosition()[0] << std::endl;
  std::cout << "Max y:                 " << graph.cgetOriginalNodes()[*std::max_element(orig_node_inds.begin(), orig_node_inds.end(),
                                                                      [&] (const size_t i, const size_t j) -> bool { return graph.cgetOriginalNodes()[i]->cgetPosition()[1] < graph.cgetOriginalNodes()[j]->cgetPosition()[1]; })]->cgetPosition()[1] << std::endl;
  std::cout << "Min y:                 " << graph.cgetOriginalNodes()[*std::min_element(orig_node_inds.begin(), orig_node_inds.end(),
                                                                      [&] (const size_t i, const size_t j) -> bool { return graph.cgetOriginalNodes()[i]->cgetPosition()[1] < graph.cgetOriginalNodes()[j]->cgetPosition()[1]; })]->cgetPosition()[1] << std::endl;

  // Pick hotspot
  std::vector<uint32_t> hotspot_inds;
  hotspot_inds.reserve(NUM_HOTSPOTS);
  if(NUM_HOTSPOTS >= 2)
  {
    hotspot_inds.emplace_back(*std::min_element(std::execution::par_unseq, orig_node_inds.begin(), orig_node_inds.end(),
                               [&] (const uint32_t i, const uint32_t j) -> bool
                               {
                                 return graph.cgetOriginalNodes()[i]->cgetPosition()[2] < graph.cgetOriginalNodes()[j]->cgetPosition()[2];
                               }));
    hotspot_inds.emplace_back(*std::max_element(std::execution::par_unseq, orig_node_inds.begin(), orig_node_inds.end(),
                               [&] (const uint32_t i, const uint32_t j) -> bool
                               {
                                 return graph.cgetOriginalNodes()[i]->cgetPosition()[2] < graph.cgetOriginalNodes()[j]->cgetPosition()[2];
                               }));
  }
  size_t seed = 54;
  do
  {
    std::sample(orig_node_inds.begin(), orig_node_inds.end(), std::back_inserter(hotspot_inds), NUM_HOTSPOTS - hotspot_inds.size(), std::mt19937{seed++});
    std::sort(std::execution::par_unseq, hotspot_inds.begin(), hotspot_inds.end());
    hotspot_inds.erase(std::unique(std::execution::par_unseq, hotspot_inds.begin(), hotspot_inds.end()), hotspot_inds.end());
  }
  while(hotspot_inds.size() < NUM_HOTSPOTS);

  // Make event points
  Eigen::Matrix<uint32_t,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> event_points;
  std::sample(orig_node_inds.begin(), orig_node_inds.end(), event_points.begin(), NUM_EVENT_POINTS, std::mt19937{43});


  std::cout << "Hotspots Table:\nx,y\\\\\n";
  std::for_each(hotspot_inds.cbegin(), hotspot_inds.cend(),
  [&] (const size_t hs_ind) -> void
  {
    std::cout << std::fixed
              << std::setprecision(std::numeric_limits<float>::max_digits10)
              << graph.cgetOriginalNodes()[hs_ind]->cgetPosition()[0]
              << ","
              << graph.cgetOriginalNodes()[hs_ind]->cgetPosition()[1]
              << "\\\\\n";
  });
  std::cout << std::endl;

  std::cout << "Event Points Table:\nx,y\\\\\n";
  std::for_each(event_points.cbegin(), event_points.cend(),
  [&] (const size_t ep_ind) -> void
  {
    std::cout << std::fixed
              << std::setprecision(std::numeric_limits<float>::max_digits10)
              << graph.cgetOriginalNodes()[ep_ind]->cgetPosition()[0]
              << ","
              << graph.cgetOriginalNodes()[ep_ind]->cgetPosition()[1]
              << "\\\\\n";
  });
  std::cout << std::endl;



//  matplotlibcpp::figure();
//  matplotlibcpp::title("Original");
//  for(size_t edge_ind = 0; edge_ind < graph.numberOriginalEdges(); ++edge_ind)
//  {
//    matplotlibcpp::plot(std::vector<double>({graph.cgetOriginalEdges()[edge_ind]->cgetFromNode()->cgetPosition()[0],
//                                             graph.cgetOriginalEdges()[edge_ind]->cgetToNode()->  cgetPosition()[0]}),
//                        std::vector<double>({graph.cgetOriginalEdges()[edge_ind]->cgetFromNode()->cgetPosition()[1],
//                                             graph.cgetOriginalEdges()[edge_ind]->cgetToNode()->  cgetPosition()[1]}),
//                        std::map<std::string,std::string>({std::make_pair("color","k")}));
//  }
//  for(size_t node_ind = 0; node_ind < graph.numberOriginalNodes(); ++node_ind)
//  {
//    matplotlibcpp::plot(std::vector<double>({graph.cgetOriginalNodes()[node_ind]->cgetPosition()[0]}),
//                        std::vector<double>({graph.cgetOriginalNodes()[node_ind]->cgetPosition()[1]}),
//                        "b.");
//  }
//  matplotlibcpp::plot(std::vector<double>({graph.cgetRootNode().cgetPosition()[0]}),
//                      std::vector<double>({graph.cgetRootNode().cgetPosition()[1]}),
//                      "g*");

  matplotlibcpp::figure();
  matplotlibcpp::set_aspect_equal();
//  matplotlibcpp::title("Simplified");
//  for(size_t edge_ind = 0; edge_ind < graph.numberOriginalEdges(); ++edge_ind)
//  {
//    matplotlibcpp::plot(std::vector<float>({graph.cgetOriginalEdges()[edge_ind]->cgetFromNode()->cgetPosition()[0],
//                                            graph.cgetOriginalEdges()[edge_ind]->cgetToNode()->  cgetPosition()[0]}),
//                        std::vector<float>({graph.cgetOriginalEdges()[edge_ind]->cgetFromNode()->cgetPosition()[1],
//                                            graph.cgetOriginalEdges()[edge_ind]->cgetToNode()->  cgetPosition()[1]}),
//                        std::map<std::string,std::string>({std::make_pair("color","k")}));
//  }
  if(true)
  {
    // Event points
    std::for_each(event_points.cbegin(), event_points.cend(),
    [&] (const size_t ep_ind) -> void
    {
      matplotlibcpp::plot(std::vector<double>({graph.cgetOriginalNodes()[ep_ind]->cgetPosition()[0]}),
                          std::vector<double>({graph.cgetOriginalNodes()[ep_ind]->cgetPosition()[1]}),
                          std::map<std::string,std::string>({std::make_pair("color", "xkcd:sky blue"),
                                                             std::make_pair("marker", "X"),
                                                             std::make_pair("markersize", "15"),
                                                             std::make_pair("markeredgecolor", "k"),
                                                             std::make_pair("markeredgewidth", "1")}));
    });
    // Hotspots
    std::for_each(hotspot_inds.cbegin(), hotspot_inds.cend(),
    [&] (const size_t hs_ind) -> void
    {
      matplotlibcpp::plot(std::vector<double>({graph.cgetOriginalNodes()[hs_ind]->cgetPosition()[0]}),
                          std::vector<double>({graph.cgetOriginalNodes()[hs_ind]->cgetPosition()[1]}),
                          std::map<std::string,std::string>({std::make_pair("color", "r"),
                                                             std::make_pair("marker", "o"),
                                                             std::make_pair("markersize", "15"),
                                                             std::make_pair("markeredgecolor", "k"),
                                                             std::make_pair("markeredgewidth", "1")}));
    });
    // Depot
    matplotlibcpp::plot(std::vector<float>({graph.cgetRootNode().cgetPosition()[0]}),
                        std::vector<float>({graph.cgetRootNode().cgetPosition()[1]}),
                        std::map<std::string,std::string>({std::make_pair("color","g"),
                                                           std::make_pair("marker", "*"),
                                                           std::make_pair("markersize", "25"),
                                                           std::make_pair("markeredgecolor", "k"),
                                                           std::make_pair("markeredgewidth", "1")}));
  }
  // Streets
  const std::map<std::string,std::string> line_args({std::make_pair("color","k"), std::make_pair("linewidth", "0.5")});
  const double                            arrow_length = 30;
  for(size_t edge_ind = 0; edge_ind < graph.numberSimplifiedEdges(); ++edge_ind)
  {
    Eigen::Matrix<float,3,Eigen::Dynamic> vec;
    graph.cgetSimplifiedEdges()[edge_ind]->generateDiscreetSteps(1, vec);
    matplotlibcpp::plot(toVec(vec.row(0)), toVec(vec.row(1)), line_args);
//    Eigen::Matrix<double,2,1> end_unit_vec;
//    end_unit_vec[0] = vec(0, Eigen::last) - vec(0, Eigen::last - 1);
//    end_unit_vec[1] = vec(1, Eigen::last) - vec(1, Eigen::last - 1);
//    end_unit_vec.normalize();
//    const double end_angle = std::atan2(end_unit_vec[1], end_unit_vec[0]);
//    double arrow_angle = end_angle + (double(7.0) * double(M_PI) / double(8.0));
//    matplotlibcpp::plot(std::vector<float>({vec(0, Eigen::last), vec(0, Eigen::last) + float(arrow_length * std::cos(arrow_angle))}),
//                        std::vector<float>({vec(1, Eigen::last), vec(1, Eigen::last) + float(arrow_length * std::sin(arrow_angle))}),
//                        line_args);
//    arrow_angle = end_angle - (double(7.0) * double(M_PI) / double(8.0));
//    matplotlibcpp::plot(std::vector<float>({vec(0, Eigen::last), vec(0, Eigen::last) + float(arrow_length * std::cos(arrow_angle))}),
//                        std::vector<float>({vec(1, Eigen::last), vec(1, Eigen::last) + float(arrow_length * std::sin(arrow_angle))}),
//                        line_args);
  }
//  for(size_t node_ind = 0; node_ind < graph.numberSimplifiedNodes(); ++node_ind)
//  {
//    matplotlibcpp::plot(std::vector<float>({graph.cgetSimplifiedNodes()[node_ind]->cgetPosition()[0]}),
//                        std::vector<float>({graph.cgetSimplifiedNodes()[node_ind]->cgetPosition()[1]}),
//                        std::map<std::string,std::string>({std::make_pair("color","k")}));
//  }

  matplotlibcpp::show();
  exit(EXIT_SUCCESS);
}

