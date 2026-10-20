// build.rs – required by esp-idf-sys / embuild
fn main() {
    embuild::espidf::sysenv::output();
}
