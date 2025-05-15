#include <fstream>

#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_container_manager.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

// ds arbitrary test objects
constexpr int test_value = 42;
class TestConfiguration : public Configurable {
public:
  PropertyInt param_test = PropertyInt("parameter", "test", this, -1, nullptr);
};
class TestModule : public Configurable {
public:
  PropertyConfigurable_<TestConfiguration> param_test_config =
    PropertyConfigurable_<TestConfiguration>("configurable",
                                             "test",
                                             this,
                                             nullptr,
                                             nullptr);
};
class TestSystem : public Configurable {
public:
  PropertyConfigurable_<TestModule> param_test_module =
    PropertyConfigurable_<TestModule>("module", "test", this, nullptr, nullptr);
};

TEST(WriteAndRead, Parameter) {
  {
    // ds write configurable to disk
    PropertyContainerManager manager;
    std::shared_ptr<TestConfiguration> test_configuration =
      manager.create<TestConfiguration>("test");
    ASSERT_NOTNULL(test_configuration);
    ASSERT_EQ(test_configuration->param_test.value(), -1);
    test_configuration->param_test.setValue(test_value);
    manager.write("test_param.conf");
  }

  // ds verify correct write the hard way
  std::ifstream configuration_file("test_param.conf");
  ASSERT_TRUE(configuration_file.is_open());
  ASSERT_TRUE(configuration_file.good());
  std::vector<std::string> line_buffers(7);
  for (std::string& line_buffer : line_buffers) {
    std::getline(configuration_file, line_buffer);
    ASSERT_NE(configuration_file.rdstate(), configuration_file.failbit);
  }
  ASSERT_NE(line_buffers[2].find("\"name\" : \"test\""), std::string::npos);
  ASSERT_NE(line_buffers[5].find("\"parameter\" : 42"), std::string::npos);

  {
    // ds read configurable from disk again
    PropertyContainerManager manager;
    manager.read("test_param.conf");
    std::shared_ptr<TestConfiguration> test_class =
      manager.getByName<TestConfiguration>("test");
    ASSERT_NOTNULL(test_class);

    // ds verify the state of the configurable
    ASSERT_EQ(test_class->param_test.value(), test_value);
  }
}

TEST(WriteAndRead, Module) {
  {
    // ds write configurable to disk
    PropertyContainerManager manager;
    std::shared_ptr<TestModule> test_module =
      manager.create<TestModule>("test");
    ASSERT_NOTNULL(test_module);
    ASSERT_NULL(test_module->param_test_config.value());
    std::shared_ptr<TestConfiguration> test_configuration =
      manager.create<TestConfiguration>("test_configuration");
    ASSERT_NOTNULL(test_configuration);
    test_module->param_test_config.setValue(test_configuration);
    ASSERT_NOTNULL(test_module->param_test_config.value());
    ASSERT_EQ(test_module->param_test_config->param_test.value(), -1);
    test_module->param_test_config->param_test.setValue(test_value);
    manager.write("test_module.conf");
  }

  // ds verify correct write the hard way
  std::ifstream configuration_file("test_module.conf");
  ASSERT_TRUE(configuration_file.is_open());
  ASSERT_TRUE(configuration_file.good());
  std::vector<std::string> line_buffers(7);
  for (std::string& line_buffer : line_buffers) {
    std::getline(configuration_file, line_buffer);
    ASSERT_NE(configuration_file.rdstate(), configuration_file.failbit);
  }
  ASSERT_NE(line_buffers[2].find("\"name\" : \"test_configuration\""),
            std::string::npos);
  ASSERT_NE(line_buffers[5].find("\"parameter\" : 42"), std::string::npos);

  {
    // ds read configurable from disk again
    PropertyContainerManager manager;
    manager.read("test_module.conf");
    std::shared_ptr<TestModule> test_module =
      manager.getByName<TestModule>("test");
    ASSERT_NOTNULL(test_module);

    // ds verify the state of the configurable
    ASSERT_NOTNULL(test_module->param_test_config.value());
    ASSERT_EQ(test_module->param_test_config->param_test.value(), test_value);
  }
}

TEST(WriteAndRead, System) {
  {
    // ds write configurable to disk
    PropertyContainerManager manager;
    std::shared_ptr<TestSystem> test_system =
      manager.create<TestSystem>("test");
    ASSERT_NOTNULL(test_system);
    ASSERT_NULL(test_system->param_test_module.value());
    std::shared_ptr<TestModule> test_module =
      manager.create<TestModule>("test_module");
    ASSERT_NOTNULL(test_module);
    ASSERT_NULL(test_module->param_test_config.value());
    std::shared_ptr<TestConfiguration> test_configuration =
      manager.create<TestConfiguration>("test_configuration");
    ASSERT_NOTNULL(test_configuration);
    test_module->param_test_config.setValue(test_configuration);
    ASSERT_NOTNULL(test_module->param_test_config.value());
    ASSERT_EQ(test_module->param_test_config->param_test.value(), -1);
    test_module->param_test_config->param_test.setValue(test_value);
    test_system->param_test_module.setValue(test_module);
    manager.write("test_system.conf");
  }

  // ds verify correct write the hard way
  std::ifstream configuration_file("test_module.conf");
  ASSERT_TRUE(configuration_file.is_open());
  ASSERT_TRUE(configuration_file.good());
  std::vector<std::string> line_buffers(7);
  for (std::string& line_buffer : line_buffers) {
    std::getline(configuration_file, line_buffer);
    ASSERT_NE(configuration_file.rdstate(), configuration_file.failbit);
  }
  ASSERT_NE(line_buffers[2].find("\"name\" : \"test_configuration\""),
            std::string::npos);
  ASSERT_NE(line_buffers[5].find("\"parameter\" : 42"), std::string::npos);

  {
    // ds read configurable from disk again
    PropertyContainerManager manager;
    manager.read("test_system.conf");
    std::shared_ptr<TestSystem> test_system =
      manager.getByName<TestSystem>("test");
    ASSERT_NOTNULL(test_system);

    // ds verify the state of the configurable
    ASSERT_NOTNULL(test_system->param_test_module.value());
    ASSERT_NOTNULL(test_system->param_test_module->param_test_config.value());
    ASSERT_EQ(
      test_system->param_test_module->param_test_config->param_test.value(),
      test_value);
  }
}

int main(int argc_, char** argv_) {
  BOSS_REGISTER_CLASS(TestConfiguration);
  BOSS_REGISTER_CLASS(TestModule);
  BOSS_REGISTER_CLASS(TestSystem);
  return srrg2_test::runTests(argc_, argv_);
}
