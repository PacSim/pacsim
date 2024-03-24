#include "configParser.hpp"

NodeType::value ConfigElement::getType() { return this->type; }

Node ConfigElement::getNode() { return this->node; }

vector<ConfigElement> ConfigElement::getElements()
{
    if (this->type != NodeType::Sequence)
    {
        throw runtime_error("Not a sequence but a " + to_string(this->type));
    }

    vector<ConfigElement> res;
    for (const_iterator it = this->node.begin(); it != this->node.end(); ++it)
    {
        Node child = *it;
        res.push_back(ConfigElement(child));
    }
    return res;
}

ConfigElement ConfigElement::getElement(string elementName)
{
    if (this->type != NodeType::Map)
    {
        throw runtime_error("Not a map but a " + to_string(this->type));
    }

    if (!this->hasElement(elementName))
    {
        throw runtime_error(elementName + " is not a member of this config item");
    }
    return this->node[elementName];
}

bool ConfigElement::hasElement(string elementName)
{
    if (this->type != NodeType::Map && this->type != NodeType::Scalar)
    {
        return false;
    }
    if (!this->node[elementName])
    {
        return false;
    }
    return true;
}

bool ConfigElement::getElement(ConfigElement* element, string elementName)
{
    if (this->type != NodeType::Map && this->type != NodeType::Scalar)
    {
        return false;
    }
    *element = this->node[elementName];
    return true;
}

bool ConfigElement::getElements(vector<ConfigElement>* vec)
{
    if (this->type != NodeType::Sequence)
    {
        return false;
    }
    for (const_iterator it = this->node.begin(); it != this->node.end(); ++it)
    {
        Node child = *it;
        vec->push_back(ConfigElement(child));
    }
    return true;
}
