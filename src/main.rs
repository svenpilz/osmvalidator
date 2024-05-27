use clap::Parser;
use std::fs::File;

mod validator;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    file: String,
}

fn main() -> Result<(), osmpbfreader::error::Error> {
    let args = Args::parse();

    println!("Loading OSM file.");
    let mut reader = osmpbfreader::OsmPbfReader::new(File::open(args.file)?);
    println!("Validate.");
    let issues = validator::validate_turn_lanes(&mut reader)?;

    println!("Found {} issues.\nReport:\n", issues.len());
    println!("id, edit, message");
    for issue in &issues {
        println!(
                "\"{id}\", \"https://www.openstreetmap.org/edit?way={id}\", \"{message}\", \"{details}\"",
                id = issue.id.inner_id(),
                message = issue.message, details=issue.details
            );
    }

    Ok(())
}
