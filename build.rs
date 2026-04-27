use std::path::Path;

fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();

    // SDL2 main library
    let sdl2_lib_dir = std::env::var("SDL2_LIB_DIR").unwrap_or_else(|_| {
        let local_path = Path::new(&manifest_dir).join("sdl2-tmp/SDL2-2.30.11/lib/x64");
        if local_path.exists() {
            return local_path.to_string_lossy().to_string();
        }
        String::new()
    });

    if !sdl2_lib_dir.is_empty() {
        println!("cargo:rustc-link-search=native={}", sdl2_lib_dir);
    }

    // SDL2_image library
    let image_lib_dir = Path::new(&manifest_dir).join("sdl2-tmp/SDL2_image-2.8.4/lib/x64");
    if image_lib_dir.exists() {
        println!(
            "cargo:rustc-link-search=native={}",
            image_lib_dir.to_string_lossy()
        );
    }
}
