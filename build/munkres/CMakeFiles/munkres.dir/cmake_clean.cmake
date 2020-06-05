file(REMOVE_RECURSE
  "libmunkres.a"
  "libmunkres.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/munkres.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
