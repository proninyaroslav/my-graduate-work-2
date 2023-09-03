use navigation::config::Config;
use navigation::run;

fn main() {
    std::process::exit(match run(Config::from_args()) {
        Ok(_) => 0,
        Err(e) => {
            eprintln!("{}", e);
            -1
        }
    })
}
