
#ifndef _TextureManager_H_
#define _TextureManager_H_

#include <list>

class TextureManager {
  public:

    static TextureManager& instance(void);

    unsigned loadTexture(unsigned char *data, unsigned width, unsigned height);

    void deleteTexture(unsigned tex_id);
    void bindTexture(unsigned tex_id);

  private:

    std::list<unsigned> loaded_textures;

    // Private constructors since this is a singleton
    TextureManager();
    TextureManager(const TextureManager&);
    TextureManager& operator=(const TextureManager&);
    ~TextureManager();

};

#endif

