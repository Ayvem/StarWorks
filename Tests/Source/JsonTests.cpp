// ============================================================================
// JsonTests.cpp — the in-house JSON reader/writer that carries every
// .swpart file (and future data assets).
// ============================================================================

#include "TestFramework.hpp"

#include <Core/Json.hpp>

#include <cmath>

using namespace sw;

SW_TEST(JsonParsesTypicalDocument)
{
    const char* text = R"({
        "name": "FT-16 \"Fuel\" Tank",
        "id": 42,
        "mass": 1.25e3,
        "negative": -0.5,
        "flag": true,
        "nothing": null,
        "vec": [1, 2.5, -3],
        "nested": {"inner": [{"deep": false}]}
    })";
    std::string error;
    const json::Value root = json::parse(text, error);
    SW_CHECK(error.empty());
    SW_CHECK(root.isObject());
    SW_CHECK(root.string("name") == "FT-16 \"Fuel\" Tank");
    SW_CHECK_EQ(static_cast<int>(root.number("id")), 42);
    SW_CHECK(std::abs(root.number("mass") - 1250.0) < 1.0e-9);
    SW_CHECK(std::abs(root.number("negative") + 0.5) < 1.0e-12);
    SW_CHECK(root.boolean("flag"));
    SW_CHECK(root.find("nothing")->isNull());
    SW_CHECK_EQ(root.find("vec")->asArray().size(), static_cast<usize>(3));
    SW_CHECK(std::abs(root.find("vec")->asArray()[1].asNumber() - 2.5) < 1.0e-12);
    const json::Value* nested = root.find("nested");
    SW_CHECK(nested != nullptr && nested->isObject());
    SW_CHECK(!nested->find("inner")->asArray()[0].boolean("deep", true));
}

SW_TEST(JsonRejectsMalformedInput)
{
    std::string error;
    (void)json::parse("{\"a\": }", error);
    SW_CHECK(!error.empty());
    error.clear();
    (void)json::parse("[1, 2", error);
    SW_CHECK(!error.empty());
    error.clear();
    (void)json::parse("{\"a\": 1} trailing", error);
    SW_CHECK(!error.empty());
    error.clear();
    (void)json::parse("{\"a\": 12.3.4}", error);
    SW_CHECK(!error.empty());
}

SW_TEST(JsonRoundTripsThroughSerialize)
{
    json::Value root = json::Value::makeObject();
    root.set("title", json::Value("Part \"Studio\""));
    root.set("count", json::Value(9));
    root.set("scale", json::Value(0.125));
    root.set("enabled", json::Value(true));
    json::Value list = json::Value::makeArray();
    list.push(json::Value(1.0));
    list.push(json::Value(-2.5));
    root.set("values", std::move(list));
    json::Value child = json::Value::makeObject();
    child.set("x", json::Value(3.0));
    root.set("child", std::move(child));

    const std::string text = json::serialize(root);
    std::string error;
    const json::Value reparsed = json::parse(text, error);
    SW_CHECK(error.empty());
    SW_CHECK(reparsed.string("title") == "Part \"Studio\"");
    SW_CHECK_EQ(static_cast<int>(reparsed.number("count")), 9);
    SW_CHECK(std::abs(reparsed.number("scale") - 0.125) < 1.0e-12);
    SW_CHECK(reparsed.boolean("enabled"));
    SW_CHECK(std::abs(reparsed.find("values")->asArray()[1].asNumber() + 2.5) < 1.0e-12);
    SW_CHECK(std::abs(reparsed.find("child")->number("x") - 3.0) < 1.0e-12);
    // set() replaces in place (no duplicate keys).
    json::Value replaced = reparsed;
    replaced.set("count", json::Value(10));
    usize countKeys = 0;
    for (const auto& [key, value] : replaced.asObject())
    {
        countKeys += (key == "count") ? 1 : 0;
    }
    SW_CHECK_EQ(countKeys, static_cast<usize>(1));
}
