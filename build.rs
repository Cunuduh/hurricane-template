use std::{
    env, fs,
    f64::consts::PI,
    io::Write,
    path::{Path, PathBuf},
};

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, Default)]
struct Pose {
    x: f64,
    y: f64,
    #[serde(default = "f64_nan")]
    heading: f64,
}
fn f64_nan() -> f64 {
    f64::NAN
}
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, Default)]
struct PoseSettings {
    #[serde(default)]
    is_reversed: bool,
    #[serde(default = "default_voltage")]
    max_voltage: f64,
    #[serde(default)]
    fast: bool,
}
fn default_voltage() -> f64 {
    6.0
}
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
enum Action {
    DriveCurve(Vec<(Pose, PoseSettings)>),
    DrivePtp(Vec<(Pose, PoseSettings)>),
    DriveToPoint(Pose, PoseSettings),
    DriveStraight(f64, PoseSettings),
    TurnToPoint(Pose, PoseSettings),
    TurnToAngle(f64, PoseSettings),
    DriveSwing(f64, f64, f64, PoseSettings),
    TriggerOnIndex(usize, String),
    TriggerOnDistance(f64, String),
    TriggerOnAngle(f64, String),
    TriggerNow(String),
    SetPose(Pose),
    Wait(u64),
    DriveFor(u64, f64, bool),
}
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
struct Routine {
    name: String,
    actions: Vec<Action>,
}

fn deg_to_rad(deg: Option<f64>) -> f64 {
    deg.map(|d| d * PI / 180.0).unwrap_or(f64::NAN)
}
fn make_pose(x: f64, y: f64, heading_deg: Option<f64>) -> Pose {
    Pose { x, y, heading: deg_to_rad(heading_deg) }
}
fn make_settings(is_reversed: bool, max_voltage: f64, fast: bool) -> PoseSettings {
    PoseSettings { is_reversed, max_voltage, fast }
}

fn parse_number(s: &str) -> f64 {
    s.trim().parse::<f64>().expect("invalid number")
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

struct ParsedSettings {
    is_reversed: bool,
    max_voltage: f64,
    fast: bool,
}
fn parse_attrs(rest: &str) -> ParsedSettings {
    let mut settings = ParsedSettings {
        is_reversed: false,
        max_voltage: 6.0,
        fast: false,
    };
    for token in rest.split_whitespace() {
        if let Some(idx) = token.find('=') {
            let key = &token[..idx];
            let value = &token[idx + 1..];
            if key == "max_voltage" {
                settings.max_voltage = parse_number(value);
            }
        } else {
            match token {
                "fast" => settings.fast = true,
                "reverse" => settings.is_reversed = true,
                _ => {}
            }
        }
    }
    settings
}

fn parse_routine(name: &str, content: &str) -> Routine {
    let mut lines = content
        .lines()
        .map(|l| l.trim())
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .peekable();
    let mut actions: Vec<Action> = Vec::new();

    while let Some(line) = lines.next() {
        if line.starts_with("drive_curve") {
            let mut points: Vec<(Pose, PoseSettings)> = Vec::new();
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
                points.push((make_pose(x, y, h), make_settings(s.is_reversed, s.max_voltage, s.fast)));
            }
            actions.push(Action::DriveCurve(points));
        } else if line.starts_with("drive_ptp") {
            let mut points: Vec<(Pose, PoseSettings)> = Vec::new();
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
                points.push((make_pose(x, y, h), make_settings(s.is_reversed, s.max_voltage, s.fast)));
            }
            actions.push(Action::DrivePtp(points));
        } else if line.starts_with("drive_for") {
            let after = line.trim_start_matches("drive_for").trim();
            let mut parts = after.splitn(2, ' ');
            let ms: u64 = parts
                .next()
                .unwrap_or("0")
                .parse()
                .expect("milliseconds");
            let attrs = parts.next().unwrap_or("");
            let s = parse_attrs(attrs);
            actions.push(Action::DriveFor(ms, s.max_voltage, s.is_reversed));
        } else if line.starts_with("drive_to ")
            || line.starts_with("drive_to(")
            || line.starts_with("drive_to\t")
        {
            let after = line.trim_start_matches("drive_to").trim();
            let lp = after.find('(').expect("(");
            let rp = after.find(')').expect(")");
            let (x, y, h) = parse_tuple(&after[lp..=rp]);
            let rest = &after[rp + 1..];
            let s = parse_attrs(rest);
            actions.push(Action::DriveToPoint(make_pose(x, y, h), make_settings(s.is_reversed, s.max_voltage, s.fast)));
        } else if line.starts_with("drive_straight") {
            let after = line.trim_start_matches("drive_straight").trim();
            let mut parts = after.splitn(2, ' ');
            let dist_str = parts.next().unwrap_or("");
            let s = parse_attrs(parts.next().unwrap_or(""));
            let dist = parse_number(dist_str);
            actions.push(Action::DriveStraight(dist, make_settings(s.is_reversed, s.max_voltage, s.fast)));
        } else if line.starts_with("turn_to_point") || line.starts_with("turn_to ") {
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
            actions.push(Action::TurnToPoint(make_pose(x, y, h), make_settings(s.is_reversed, s.max_voltage, s.fast)));
        } else if line.starts_with("turn_to_angle") {
            let after = line.trim_start_matches("turn_to_angle").trim();
            let mut parts = after.splitn(2, ' ');
            let ang_deg = parse_number(parts.next().unwrap_or("0"));
            let s = parse_attrs(parts.next().unwrap_or(""));
            let ang_rad = ang_deg * PI / 180.0;
            actions.push(Action::TurnToAngle(ang_rad, make_settings(s.is_reversed, s.max_voltage, s.fast)));
        } else if line.starts_with("drive_swing") {
            let after = line.trim_start_matches("drive_swing").trim();
            let mut ang_deg = 0.0;
            let mut left_v = 0.0;
            let mut right_v = 0.0;
            let mut fast = false;
            for (i, token) in after.split_whitespace().enumerate() {
                if let Some(idx) = token.find('=') {
                    let key = &token[..idx];
                    let value = &token[idx + 1..];
                    match key {
                        "left_voltage" => left_v = parse_number(value),
                        "right_voltage" => right_v = parse_number(value),
                        _ => {}
                    }
                } else if token == "fast" {
                    fast = true;
                } else if i == 0 {
                    ang_deg = parse_number(token);
                }
            }
            let ang_rad = ang_deg * PI / 180.0;
            actions.push(Action::DriveSwing(ang_rad, left_v, right_v, make_settings(false, 6.0, fast)));
        } else if line.starts_with("trigger_on_distance") {
            let after = line.trim_start_matches("trigger_on_distance").trim();
            let mut parts = after.splitn(2, ' ');
            let d = parse_number(parts.next().unwrap_or("0"));
            let name = parts.next().unwrap_or("").trim();
            let name = name.trim_matches('"').to_string();
            actions.push(Action::TriggerOnDistance(d, name));
        } else if line.starts_with("trigger_on_index") {
            let after = line.trim_start_matches("trigger_on_index").trim();
            let mut parts = after.splitn(2, ' ');
            let idx: usize = parts.next().unwrap_or("0").parse().expect("index");
            let name = parts.next().unwrap_or("").trim();
            let name = name.trim_matches('"').to_string();
            actions.push(Action::TriggerOnIndex(idx, name));
        } else if line.starts_with("trigger_on_angle") {
            let after = line.trim_start_matches("trigger_on_angle").trim();
            let mut parts = after.splitn(2, ' ');
            let a_deg = parse_number(parts.next().unwrap_or("0"));
            let name = parts.next().unwrap_or("").trim();
            let name = name.trim_matches('"').to_string();
            let a_rad = a_deg * PI / 180.0;
            actions.push(Action::TriggerOnAngle(a_rad, name));
        } else if line.starts_with("trigger_now") {
            let after = line.trim_start_matches("trigger_now").trim();
            let name = after.trim().trim_matches('"').to_string();
            actions.push(Action::TriggerNow(name));
        } else if line.starts_with("set_pose") {
            let after = line.trim_start_matches("set_pose").trim();
            let lp = after.find('(').expect("set_pose (");
            let rp = after.find(')').expect(")");
            let (x, y, h) = parse_tuple(&after[lp..=rp]);
            actions.push(Action::SetPose(make_pose(x, y, h)));
        } else if line.starts_with("wait ") {
            let after = line.trim_start_matches("wait").trim();
            let ms: u64 = after.parse().expect("milliseconds");
            actions.push(Action::Wait(ms));
        }
    }

    Routine {
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
