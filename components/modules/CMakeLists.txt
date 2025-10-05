set(embed_files "index.html.gz")

idf_component_register(EMBED_FILES ${embed_files})

component_compile_options(-ffast-math -O3)
