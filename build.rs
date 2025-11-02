use std::{
    env, fs,
    io::Write,
    path::{Path, PathBuf},
};

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
struct IRPose {
    x: f64,
    y: f64,
    heading: Option<f64>,
}
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
struct IRPoseSettings {
    is_reversed: bool,
    max_voltage: f64,
}
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
enum IRAction {
    DriveCurve(Vec<(IRPose, IRPoseSettings)>),
    DriveBoomerang(Vec<(IRPose, IRPoseSettings)>),
    DrivePtp(Vec<(IRPose, IRPoseSettings)>),
    DriveToPoint(IRPose, IRPoseSettings),
    DriveStraight(f64, IRPoseSettings),
    TurnToPoint(IRPose, IRPoseSettings),
    TurnToAngle(f64, IRPoseSettings),
    TriggerOnIndex(usize, String),
    TriggerOnDistance(f64, String),
    TriggerOnAngle(f64, String),
    TriggerNow(String),
    SetPose(IRPose),
    Wait(u64),
    DriveFor(u64, f64, bool),
}
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
struct IRRoutine {
    name: String,
    actions: Vec<IRAction>,
}

fn parse_number(s: &str) -> f64 {
    s.trim().parse::<f64>().expect("invalid number")
}
fn parse_bool(s: &str) -> bool {
    match s.trim() {
        "true" => true,
        "false" => false,
        _ => panic!("invalid bool"),
    }
}
fn parse_tuple(s: &str) -> (f64, f64, Option<f64>) {
    let t = s.trim().trim_start_matches('(').trim_end_matches(')');
    let mut parts = t.split(',').map(|p| p.trim());
    let x = parts.next().expect("x");
    let y = parts.next().expect("y");
    let h = parts.next();
    let x = parse_number(x);
    let y = parse_number(y);
    let h = h.map(parse_number);
    (x, y, h)
}

fn parse_attrs(mut rest: &str) -> IRPoseSettings {
    let mut settings = IRPoseSettings {
        is_reversed: false,
        max_voltage: 12.0,
    };
    while let Some(idx) = rest.find('=') {
        let (key_part, value_rest) = rest.split_at(idx);
        let key = key_part.split_whitespace().last().unwrap_or("");
        let value_and_more = &value_rest[1..];
        let end = value_and_more.find(' ');
        let end_idx = end.unwrap_or(value_and_more.len());
        let value = &value_and_more[..end_idx];
        match key {
            "max_voltage" => settings.max_voltage = parse_number(value),
            "reverse" => settings.is_reversed = parse_bool(value),
            _ => {}
        }
        if end.is_none() {
            break;
        }
        rest = &value_and_more[end_idx + 1..];
    }
    settings
}

fn parse_routine(name: &str, content: &str) -> IRRoutine {
    let mut lines = content
        .lines()
        .map(|l| l.trim())
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .peekable();
    let mut actions: Vec<IRAction> = Vec::new();

    while let Some(line) = lines.next() {
        if line.starts_with("drive_curve") {
            // drive_curve
            //   point (x,y[,heading]) [attrs]
            let mut points: Vec<(IRPose, IRPoseSettings)> = Vec::new();
            while let Some(peek) = lines.peek() {
                if !peek.starts_with("point") {
                    break;
                }
                let l = lines.next().unwrap();
                let after = l.trim_start_matches("point").trim();
                let lp = after.find('(').expect("point (");
                let rp = after.find(')').expect(")");
                let tuple_str = &after[lp..=rp];
                let (x, y, h) = parse_tuple(tuple_str);
                let rest = &after[rp + 1..];
                let s = parse_attrs(rest);
                let pose = IRPose { x, y, heading: h };
                points.push((pose, s));
            }
            actions.push(IRAction::DriveCurve(points));
        } else if line.starts_with("drive_boomerang") {
            // drive_boomerang
            //   point (x,y,heading) [attrs]
            // only accepts max 2 points, extras are ignored
            let mut points: Vec<(IRPose, IRPoseSettings)> = Vec::new();
            while let Some(peek) = lines.peek() {
                if !peek.starts_with("point") {
                    break;
                }
                let l = lines.next().unwrap();
                let after = l.trim_start_matches("point").trim();
                let lp = after.find('(').expect("point (");
                let rp = after.find(')').expect(")");
                let tuple_str = &after[lp..=rp];
                let (x, y, h) = parse_tuple(tuple_str);
                let rest = &after[rp + 1..];
                let s = parse_attrs(rest);
                let heading = h.expect("drive_boomerang requires heading for all points");
                let pose = IRPose {
                    x,
                    y,
                    heading: Some(heading),
                };
                points.push((pose, s));
                if points.len() >= 2 {
                    while let Some(peek) = lines.peek() {
                        // eat the rest of the points
                        if peek.starts_with("point") {
                            lines.next();
                        } else {
                            break;
                        }
                    }
                    break;
                }
            }
            actions.push(IRAction::DriveBoomerang(points));
        } else if line.starts_with("drive_ptp") {
            // drive_ptp
            //   point (x,y[,heading]) [attrs]
            let mut points: Vec<(IRPose, IRPoseSettings)> = Vec::new();
            while let Some(peek) = lines.peek() {
                if !peek.starts_with("point") {
                    break;
                }
                let l = lines.next().unwrap();
                let after = l.trim_start_matches("point").trim();
                let lp = after.find('(').expect("point (");
                let rp = after.find(')').expect(")");
                let tuple_str = &after[lp..=rp];
                let (x, y, h) = parse_tuple(tuple_str);
                let rest = &after[rp + 1..];
                let s = parse_attrs(rest);
                let pose = IRPose { x, y, heading: h };
                points.push((pose, s));
            }
            actions.push(IRAction::DrivePtp(points));
        } else if line.starts_with("drive_for") {
            // drive_for <ms> [attrs]
            let after = line.trim_start_matches("drive_for").trim();
            let mut parts = after.splitn(2, ' ');
            let ms: u64 = parts
                .next()
                .unwrap_or("0")
                .parse()
                .expect("milliseconds");
            let attrs = parts.next().unwrap_or("");
            let s = parse_attrs(attrs);
            actions.push(IRAction::DriveFor(ms, s.max_voltage, s.is_reversed));
        } else if line.starts_with("drive_to ")
            || line.starts_with("drive_to(")
            || line.starts_with("drive_to\t")
        {
            // drive_to (x,y[,heading]) [attrs]
            let after = line.trim_start_matches("drive_to").trim();
            let lp = after.find('(').expect("(");
            let rp = after.find(')').expect(")");
            let (x, y, h) = parse_tuple(&after[lp..=rp]);
            let rest = &after[rp + 1..];
            let s = parse_attrs(rest);
            let pose = IRPose { x, y, heading: h };
            actions.push(IRAction::DriveToPoint(pose, s));
        } else if line.starts_with("drive_straight") {
            // drive_straight distance [attrs]
            let after = line.trim_start_matches("drive_straight").trim();
            let mut parts = after.splitn(2, ' ');
            let dist_str = parts.next().unwrap_or("");
            let s = parse_attrs(parts.next().unwrap_or(""));
            let dist = parse_number(dist_str);
            actions.push(IRAction::DriveStraight(dist, s));
        } else if line.starts_with("turn_to_point") || line.starts_with("turn_to ") {
            // turn_to_point (x,y[,heading]) [attrs]
            let after = line
                .trim_start_matches("turn_to_point")
                .trim()
                .trim_start_matches("turn_to")
                .trim();
            let lp = after.find('(').expect("(");
            let rp = after.find(')').expect(")");
            let (x, y, h) = parse_tuple(&after[lp..=rp]);
            let rest = &after[rp + 1..];
            let s = parse_attrs(rest);
            let pose = IRPose { x, y, heading: h };
            actions.push(IRAction::TurnToPoint(pose, s));
        } else if line.starts_with("turn_to_angle") {
            // turn_to_angle angle [attrs]
            let after = line.trim_start_matches("turn_to_angle").trim();
            let mut parts = after.splitn(2, ' ');
            let ang = parse_number(parts.next().unwrap_or("0"));
            let s = parse_attrs(parts.next().unwrap_or(""));
            actions.push(IRAction::TurnToAngle(ang, s));
        } else if line.starts_with("trigger_on_distance") {
            // trigger_on_distance distance "name"
            let after = line.trim_start_matches("trigger_on_distance").trim();
            let mut parts = after.splitn(2, ' ');
            let d = parse_number(parts.next().unwrap_or("0"));
            let name = parts.next().unwrap_or("").trim();
            let name = name.trim_matches('"').to_string();
            actions.push(IRAction::TriggerOnDistance(d, name));
        } else if line.starts_with("trigger_on_index") {
            // trigger_on_index index "name"
            let after = line.trim_start_matches("trigger_on_index").trim();
            let mut parts = after.splitn(2, ' ');
            let idx: usize = parts.next().unwrap_or("0").parse().expect("index");
            let name = parts.next().unwrap_or("").trim();
            let name = name.trim_matches('"').to_string();
            actions.push(IRAction::TriggerOnIndex(idx, name));
        } else if line.starts_with("trigger_on_angle") {
            // trigger_on_angle angle "name"
            let after = line.trim_start_matches("trigger_on_angle").trim();
            let mut parts = after.splitn(2, ' ');
            let a = parse_number(parts.next().unwrap_or("0"));
            let name = parts.next().unwrap_or("").trim();
            let name = name.trim_matches('"').to_string();
            actions.push(IRAction::TriggerOnAngle(a, name));
        } else if line.starts_with("trigger_now") {
            // trigger_now "name"
            let after = line.trim_start_matches("trigger_now").trim();
            let name = after.trim().trim_matches('"').to_string();
            actions.push(IRAction::TriggerNow(name));
        } else if line.starts_with("set_pose") {
            // set_pose (x,y[,heading])
            let after = line.trim_start_matches("set_pose").trim();
            let lp = after.find('(').expect("set_pose (");
            let rp = after.find(')').expect(")");
            let (x, y, h) = parse_tuple(&after[lp..=rp]);
            let pose = IRPose { x, y, heading: h };
            actions.push(IRAction::SetPose(pose));
        } else if line.starts_with("wait ") {
            // wait milliseconds
            let after = line.trim_start_matches("wait").trim();
            let ms: u64 = after.parse().expect("milliseconds");
            actions.push(IRAction::Wait(ms));
        } else {
            // unknown line types are ignored
        }
    }

    IRRoutine {
        name: name.to_string(),
        actions,
    }
}

fn find_routine_files(dir: &Path) -> Vec<PathBuf> {
    let mut res = Vec::new();
    if let Ok(read) = fs::read_dir(dir) {
        for e in read.flatten() {
            let p = e.path();
            if p.extension().and_then(|s| s.to_str()) == Some("routine") {
                if let Some(name) = p.file_name().and_then(|s| s.to_str())
                    && (name.starts_with('.') || name.starts_with("._"))
                {
                    continue;
                }
                res.push(p);
            }
        }
    }
    res
}

fn main() {
    slint_build::compile_with_config(
        "src/ui/ui.slint",
        slint_build::CompilerConfiguration::new()
            .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer),
    )
    .expect("Slint build failed");

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let routines_dir = Path::new("src/routines");
    println!("cargo:rerun-if-changed=src/routines");

    let files = find_routine_files(routines_dir);
    let mut index_src = String::from("pub static ROUTINE_BLOBS: &[(&str, &[u8])] = &[\n");
    for file in files {
        let name = file.file_stem().unwrap().to_string_lossy().to_string();
        println!("cargo:rerun-if-changed={}", file.display());
        let content = fs::read_to_string(&file).expect("read .routine");
        let routine = parse_routine(&name, &content);
        let bytes = postcard::to_allocvec(&routine).expect("serialize routine");
        let out_file = out_dir.join(format!("routine_{name}.bin"));
        fs::write(&out_file, &bytes).expect("write bin");
        index_src.push_str(&format!(
            "    (\"{}\", include_bytes!(concat!(env!(\"OUT_DIR\"), \"/{}\")) as &[u8]),\n",
            name,
            out_file.file_name().unwrap().to_string_lossy()
        ));
    }
    index_src.push_str("];");
    let mut f = fs::File::create(out_dir.join("routines_index.rs")).expect("create index");
    f.write_all(index_src.as_bytes()).unwrap();
}
