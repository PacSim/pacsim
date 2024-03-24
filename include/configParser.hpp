#ifndef CONFIGPARSER_HPP
#define CONFIGPARSER_HPP

#include "rclcpp/rclcpp.hpp"
#include "types.hpp"
#include "yaml-cpp/yaml.h"
#include <memory>
#include <string>
#include <vector>

using namespace std;
using namespace YAML;

class ConfigElement
{
public:
    ConfigElement(Node node)
        : node(node)
        , type(node.Type())
    {
    }

    NodeType::value getType();
    Node getNode();
    bool hasElement(string elementName);
    vector<ConfigElement> getElements();
    ConfigElement getElement(string elementName);
    bool getElement(ConfigElement* element, string elementName);
    bool getElements(vector<ConfigElement>* vec);
    ConfigElement operator[](string elementName)
    {
        if (this->type != NodeType::Map)
        {
            throw runtime_error("Not a map but a " + to_string(this->type));
        }
        return this->node[elementName];
    }
    ConfigElement operator[](int i)
    {
        if (this->type != NodeType::Sequence)
        {
            throw runtime_error("Not a sequence but a " + to_string(this->type));
        }
        return this->node[i];
    }
    template <typename T> inline T getElement(string elementName) { return this->node[elementName].as<T>(); };
    template <typename T> inline bool getElement(T* res, string elementName)
    {
        *res = this->node[elementName].as<T>();
        return true;
    }

protected:
    Node node;
    NodeType::value type;
};

class Config : public ConfigElement
{
public:
    Config(string path)
        : ConfigElement(LoadFile(path))
    {
    }
};

#endif // CONFIGPARSER_HPP