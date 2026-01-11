#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <omp.h>
#include "scene.h"
#include "random.h"
#include "bvh.h"

// Find the closest object that intersects with the ray. Returns false if there
// is no such object.
bool find_hit(struct bvh_node *root,
              struct ray *r,
              double t0, double t1, struct hit *hit) {
  return bvh_hit(root, r, t0, t1, hit);
}

// Find the colour contribution of the given ray. Uses 'find_hit' to find the
// hit, and then checks if the light scatters. If so, the ray recursively
// bounces along the scene.
struct vec colour(struct rng *rng,
                  int max_depth,
                  struct bvh_node *root,
                  struct ray *r) {
  if (max_depth <= 0) {
    return (struct vec){1,1,1};
  } else {
    struct hit hit;
    if (find_hit(root, r, 0.001, INFINITY, &hit)) {
      struct scattering s;
      if (scattering(rng, r, &hit, &s)) {
        return
          vec_mul(s.attenuation,
                  colour(rng, max_depth-1, root, &s.scattered));
      } else {
        return (struct vec){0,0,0};
      }
    } else {
      struct vec unit_direction = vec_normalise(r->direction);
      double t = 0.5 * (unit_direction.y + 1);
      return vec_add((struct vec){1-t,1-t,1-t}, (struct vec){t*0.5,t*0.7,t});
    }
  }
}

// Compute a ray traced image.
//
// * max_depth is the depth limit.
//
// * nx is the width of the generated image.
//
// * ny is the height of the generated image.
//
// * ns is the number of rays fired per pixel. A lower number results in a more
//   grainy image.
//
// * num_objects is the number of objects in the array 'objects'.
//
// * objects is an array of objects.
//
// * camera is the position of the camera.
//
// * out is a an array of size nx*ny, where the final is stored in row-major
//   order.
void render(int max_depth, int nx, int ny, int ns,
            size_t num_objects, struct object *objects,
            struct camera *camera,
            uint32_t *out) {
  struct rng rng;
  seed_rng(&rng, max_depth ^ nx ^ ny ^ ns);

  // We will tally up the color of each pixel in this array.
  struct vec *rgbs = calloc(nx*ny, sizeof(struct vec));

  // Zero-initialise the color vectors.
  for (int i = 0; i < nx*ny; i++) {
    rgbs[i] = (struct vec){0,0,0};
  }
  
  // Build BVH
  double bvh_start = omp_get_wtime();
  struct bvh_node *root = build_bvh(objects, num_objects, 0, 0, &rng);
  double bvh_end = omp_get_wtime();
  printf("BVH Construction Time: %f seconds\n", bvh_end - bvh_start);

  // Loop across all pixels.
  double render_start = omp_get_wtime();
  #pragma omp parallel for schedule(dynamic)
  for (int x = 0; x < nx; x++) {
    for (int y = 0; y < ny; y++) {
      struct rng rng;
      seed_rng(&rng, max_depth ^ nx ^ ny ^ ns ^ x ^ y);

      for (int iter = 0; iter < ns; iter++) {
        double ud = random_double(&rng);
        double vd = random_double(&rng);
        double u = (x + ud) / nx;
        double v = (y + vd) / ny;
        struct ray r = get_ray(&rng, camera, u, v);

       // Accumulate in the vector for this pixel. Reverse the Y dimension
       // because the image format coordinates assume Y goes down, while our
       // geometrical calculations assume Y goes up. We scale the colour by the
       // inverse of the number of samples.
       struct vec c = colour(&rng, max_depth, root, &r);
       rgbs[(ny-y-1)*nx+x] =
         vec_add(rgbs[(ny-y-1)*nx+x],
                 vec_scale(1.0/ns, c));
      }
    }
  }
  double render_end = omp_get_wtime();
  printf("Rendering Time: %f seconds\n", render_end - render_start);

  // Encode as RGB integers.
  for (int i = 0; i < nx*ny; i++) {
    out[i] = encode_rgb(rgbs[i]);
  }

  free(rgbs);
  free_bvh(root);
}

void ppm_to_file(char *filename, uint32_t *pixels, int height, int width) {
  FILE *file = fopen(filename, "wb");
  assert(file != NULL);

  fprintf(file, "P6\n%d %d\n255\n", width, height);

  for (int i = 0; i < height*width; i++) {
    unsigned char pixel[3];
    pixel[0] = pixels[i]>>16;
    pixel[1] = pixels[i]>>8;
    pixel[2] = pixels[i]>>0;
    fwrite(pixel, 1, 3, file);
  }

  fclose(file);
}

int main(int argc, char** argv) {
  struct vec lookfrom, lookat;
  double dist_to_focus = 10;
  double aperture = 0.01;
  double fov = 75;
  int max_depth = 5;

  assert(argc == 5 || argc == 6);

  char *out_fname = argv[1];
  int nx = atoi(argv[2]);
  int ny = atoi(argv[3]);
  int ns = atoi(argv[4]);

  const char *scene_name = "nice";

  if (argc == 6) {
    scene_name = argv[5];
  }

  size_t num_materials, num_objects;
  struct material *materials;
  struct object *objects;

  if (!scene_by_name(scene_name,
                     &lookfrom, &lookat,
                     &num_materials, &materials,
                     &num_objects, &objects)) {
    fprintf(stderr, "Unknown scene: %s\n", scene_name);
    return 1;
  }

  for (size_t i = 0; i < num_objects; i++) {
    describe_object(&objects[i]);
  }

  struct camera cam = mk_camera(lookfrom, lookat, fov,
                                (double)nx/(double)ny,
                                aperture, dist_to_focus);

  uint32_t *argbs = calloc(nx*ny, sizeof(uint32_t));

  render(max_depth, nx, ny, ns, num_objects, objects, &cam, argbs);

  double save_start = omp_get_wtime();
  ppm_to_file(out_fname, argbs, ny, nx);
  double save_end = omp_get_wtime();
  printf("File Save Time: %f seconds\n", save_end - save_start);

  free(argbs);
  free(materials);
  free(objects);

}
