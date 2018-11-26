/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PARSE_CONFIGPARSE_H_
#define INCLUDE_SCRIMMAGE_PARSE_CONFIGPARSE_H_

#include <map>
#include <vector>
#include <string>

namespace boost {
namespace property_tree {
namespace detail {
namespace rapidxml {
template <class T> class xml_node;
}}}}

namespace rapidxml = boost::property_tree::detail::rapidxml;

namespace scrimmage {

class FileSearch;

class ConfigParse {
 public:
    ConfigParse();
    void set_required(std::string node_name);
    bool parse(std::map<std::string, std::string> &overrides,
        std::string filename, std::string env_var, FileSearch &file_search,
        bool verbose = false);
    std::map<std::string, std::string> & params();
    std::string filename();
    std::string directory();
    std::string extension();
    std::string stem();
    void print_params();

 protected:
    std::map<std::string, std::string> params_;
    std::vector<std::string> required_;
    std::string filename_;

    void recursive_params(rapidxml::xml_node<char> *root,
        std::map<std::string, std::string> &overrides,
        std::map<std::string, std::string> &params,
        const std::string &prev);
};
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PARSE_CONFIGPARSE_H_
