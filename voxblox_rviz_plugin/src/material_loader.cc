#include "voxblox_rviz_plugin/material_loader.h"

#include <OgreResourceGroupManager.h>
#include <OgreMaterialManager.h>
#include <OgreDataStream.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>

namespace voxblox_rviz_plugin {

bool MaterialLoader::materials_loaded_ = false;

void MaterialLoader::loadMaterials() {
  if (materials_loaded_) {
    std::cout << "[VoxbloxMaterial] Materials already loaded, skipping." << std::endl;
    return;
  }
  
  std::cout << "[VoxbloxMaterial] Loading materials..." << std::endl;
  
  std::string package_path = ament_index_cpp::get_package_share_directory("voxblox_rviz_plugin");
  std::string material_file = package_path + "/content/materials/voxblox.material";
  
  std::cout << "[VoxbloxMaterial] Material file: " << material_file << std::endl;
  
  // Check if file exists
  std::ifstream file_check(material_file);
  if (!file_check.good()) {
    std::cerr << "[VoxbloxMaterial] ERROR: Material file does not exist!" << std::endl;
    materials_loaded_ = true;
    return;
  }
  file_check.close();
  
  try {
    // Open the file and parse the material script directly
    std::ifstream* ifs = new std::ifstream(material_file.c_str(), std::ios::binary);
    if (!ifs->is_open()) {
      std::cerr << "[VoxbloxMaterial] ERROR: Could not open material file!" << std::endl;
      delete ifs;
      materials_loaded_ = true;
      return;
    }
    
    Ogre::DataStreamPtr stream(new Ogre::FileStreamDataStream(ifs, true));
    Ogre::MaterialManager::getSingleton().parseScript(
        stream, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    std::cout << "[VoxbloxMaterial] Parsed material script" << std::endl;
    
    // Check if material exists
    if (Ogre::MaterialManager::getSingleton().resourceExists("VoxbloxMaterial", 
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)) {
      std::cout << "[VoxbloxMaterial] SUCCESS: VoxbloxMaterial found in General group!" << std::endl;
    } else {
      std::cerr << "[VoxbloxMaterial] ERROR: VoxbloxMaterial NOT found after parsing!" << std::endl;
      
      // List all materials
      std::cerr << "[VoxbloxMaterial] Listing all materials:" << std::endl;
      Ogre::ResourceManager::ResourceMapIterator it = 
          Ogre::MaterialManager::getSingleton().getResourceIterator();
      int count = 0;
      while (it.hasMoreElements()) {
        Ogre::ResourcePtr resource = it.getNext();
        std::cerr << "  - " << resource->getName() << " (group: " << resource->getGroup() << ")" << std::endl;
        count++;
      }
      std::cerr << "[VoxbloxMaterial] Total materials: " << count << std::endl;
    }
  } catch (const Ogre::Exception& e) {
    std::cerr << "[VoxbloxMaterial] Ogre exception while parsing material: " 
              << e.getFullDescription() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[VoxbloxMaterial] Exception while parsing material: " 
              << e.what() << std::endl;
  }
  
  materials_loaded_ = true;
}

}  // namespace voxblox_rviz_plugin
