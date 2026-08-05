use monstertruck_geometry::prelude::{Matrix4, Point3};
use monstertruck_io::step::save::*;
use monstertruck_modeling::*;

#[test]
fn step_model_default_measurement_context_is_unchanged() {
    let cshell = Shell::new().compress();
    let text = StepModel::from(&cshell).to_string();
    assert!(
        text.contains("#12 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );"),
        "default length unit must stay millimetre:\n{text}"
    );
    assert!(
        text.contains(
            "#15 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-6), #12, \
             'distance_accuracy_value','confusion accuracy');"
        ),
        "default accuracy must stay 1.0E-6:\n{text}"
    );
}

#[test]
fn step_model_custom_measurement_context_appears() {
    let cshell = Shell::new().compress();
    let context = StepMeasurementContext {
        length_prefix: SiPrefix::None,
        distance_accuracy_value: 1.0e-5,
    };
    let text = StepModel::from(&cshell)
        .with_measurement_context(context)
        .to_string();
    assert!(
        text.contains("SI_UNIT($,.METRE.)"),
        "custom metre unit:\n{text}"
    );
    assert!(
        text.contains("LENGTH_MEASURE(1.0E-5)"),
        "custom accuracy:\n{text}"
    );
    assert!(!text.contains("SI_UNIT(.MILLI.,.METRE.)"));
    assert!(!text.contains("1.0E-6"));
}

#[test]
fn step_models_default_measurement_context_is_unchanged() {
    let json = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../resources/shape/cube.json"
    ));
    let csolid: CompressedSolid = serde_json::from_reader(json.as_slice()).unwrap();
    let models: StepModels<_, _, _> = std::iter::once(&csolid).collect();
    let text = models.to_string();
    assert!(
        text.contains("#12 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );"),
        "default length unit must stay millimetre:\n{text}"
    );
    assert!(
        text.contains(
            "#15 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-6), #12, \
             'distance_accuracy_value','confusion accuracy');"
        ),
        "default accuracy must stay 1.0E-6:\n{text}"
    );
}

#[test]
fn step_models_custom_measurement_context_appears() {
    let json = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../resources/shape/cube.json"
    ));
    let csolid: CompressedSolid = serde_json::from_reader(json.as_slice()).unwrap();
    let context = StepMeasurementContext {
        length_prefix: SiPrefix::Micro,
        distance_accuracy_value: 2.0e-7,
    };
    let models: StepModels<_, _, _> = std::iter::once(&csolid).collect();
    let text = models.with_measurement_context(context).to_string();
    assert!(
        text.contains("SI_UNIT(.MICRO.,.METRE.)"),
        "custom micro unit:\n{text}"
    );
    assert!(
        text.contains("LENGTH_MEASURE(2.0E-7)"),
        "custom accuracy:\n{text}"
    );
}

#[test]
fn step_design_custom_measurement_context_appears() {
    let context = StepMeasurementContext {
        length_prefix: SiPrefix::None,
        distance_accuracy_value: 5.0e-4,
    };
    let design: StepDesign<Point3, Option<Point3>, Matrix4> =
        StepDesign::from_model(Point3::origin());
    let text = design.with_measurement_context(context).to_string();
    assert!(
        text.contains("SI_UNIT($,.METRE.)"),
        "custom metre unit:\n{text}"
    );
    assert!(
        text.contains("UNCERTAINTY_MEASURE_WITH_UNIT(5.0E-4,"),
        "custom accuracy:\n{text}"
    );
}
