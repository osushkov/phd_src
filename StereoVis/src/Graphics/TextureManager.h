
#ifndef _TextureManager_H_
#define _TextureManager_H_

#include <string>
#include <list>
#include <map>

class TextureManager {
  public:

    static TextureManager& instance(void);

    unsigned loadTexture(std::string filename);
    unsigned loadTexture(unsigned char *data, unsigned width, unsigned height); // Assumes RGB TODO: fix this?

    void deleteTexture(unsigned tex_id);

    void bindTexture(unsigned tex_id);

  private:

    std::list<unsigned> loaded_textures;
    std::map<std::string, unsigned> texture_map;

    // Private constructors since this is a singleton
    TextureManager();
    TextureManager(const TextureManager&);
    TextureManager& operator=(const TextureManager&);
    ~TextureManager();

};

#endif

