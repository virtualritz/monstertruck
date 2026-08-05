use super::*;
use crate::step::load::{GeometricCurveSet, GeometricSetSelect};
use crate::step::save::{
    FloatDisplay, IndexSliceDisplay, StepDisplay, StepLength, VectorAsDirection,
};

impl save::StepFormat for SurfaceCurveAssociatedGeometry {
    fn fmt(&self, idx: usize, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => curve.fmt(idx, f),
            SurfaceCurveAssociatedGeometry::Surface(surface) => surface.fmt(idx, f),
        }
    }
}

impl save::StepLength for SurfaceCurveAssociatedGeometry {
    fn step_length(&self) -> usize {
        match self {
            SurfaceCurveAssociatedGeometry::ParameterCurve(curve) => curve.step_length(),
            SurfaceCurveAssociatedGeometry::Surface(surface) => surface.step_length(),
        }
    }
}

impl save::StepFormat for SurfaceCurve3D {
    fn fmt(&self, idx: usize, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let leader_idx = idx + 1;
        let (associated_indices, _) = self.associated_geometry().iter().fold(
            (
                Vec::<usize>::with_capacity(self.associated_geometry().len()),
                leader_idx + self.leader().step_length(),
            ),
            |(mut indices, cursor), entry| {
                indices.push(cursor);
                (indices, cursor + entry.step_length())
            },
        );
        let entity = match self.kind() {
            SurfaceCurveKind::SurfaceCurve => "SURFACE_CURVE",
            SurfaceCurveKind::SeamCurve => "SEAM_CURVE",
            SurfaceCurveKind::IntersectionCurve => "INTERSECTION_CURVE",
        };
        let master_representation = match self.master_representation() {
            SurfaceCurveRepresentation::Curve3D => ".CURVE_3D.",
            SurfaceCurveRepresentation::ParameterCurve0 => ".PCURVE_S1.",
            SurfaceCurveRepresentation::ParameterCurve1 => ".PCURVE_S2.",
        };
        f.write_fmt(format_args!(
            "#{idx} = {entity}('', #{leader_idx}, {associated_geometry}, {master_representation});\n",
            associated_geometry = IndexSliceDisplay(associated_indices.iter().copied()),
        ))?;
        self.leader().fmt(leader_idx, f)?;
        self.associated_geometry()
            .iter()
            .zip(associated_indices)
            .try_for_each(|(entry, entry_idx)| entry.fmt(entry_idx, f))
    }
}

impl save::StepLength for SurfaceCurve3D {
    fn step_length(&self) -> usize {
        1 + self.leader().step_length()
            + self
                .associated_geometry()
                .iter()
                .map(save::StepLength::step_length)
                .sum::<usize>()
    }
}

impl save::StepCurve for SurfaceCurve3D {
    fn same_sense(&self) -> bool { self.leader().same_sense() }
}

impl save::ConstStepLength for Processor<Sphere, Matrix4> {
    const LENGTH: usize = Processor::<monstertruck_geometry::prelude::Sphere, Matrix4>::LENGTH;
}
impl save::StepLength for Processor<Sphere, Matrix4> {
    fn step_length(&self) -> usize { <Self as save::ConstStepLength>::LENGTH }
}
impl save::StepFormat for Processor<Sphere, Matrix4> {
    fn fmt(&self, idx: usize, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Processor::new(self.entity().0)
            .transformed(*self.transform())
            .fmt(idx, f)
    }
}

impl save::StepFormat for ElementarySurface {
    fn fmt(&self, idx: usize, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Plane(x) => x.fmt(idx, f),
            Self::Sphere(x) => x.fmt(idx, f),
            Self::ToroidalSurface(x) => x.fmt(idx, f),
            Self::CylindricalSurface(processor) => {
                let position_idx = idx + 1;
                let location_idx = idx + 2;
                let axis_idx = idx + 3;
                let ref_direction_idx = idx + 4;

                let revo = processor.entity();
                let trans = processor.transform();
                let o = trans.transform_point(revo.origin());
                let p = trans.transform_point(revo.entity_curve().0);
                let axis = trans.transform_vector(revo.axis());

                let location = StepDisplay::new(o, location_idx);
                let direction_axis = VectorAsDirection(axis);
                let axis = StepDisplay::new(direction_axis, axis_idx);
                let raw_ref_direction = VectorAsDirection((p - o).normalize());
                let ref_direction = StepDisplay::new(raw_ref_direction, ref_direction_idx);
                let radius = (p - o).magnitude();

                f.write_fmt(format_args!(
                    "#{idx} = CYLINDRICAL_SURFACE('', #{position_idx}, {radius});
#{position_idx} = AXIS2_PLACEMENT_3D('', #{location_idx}, #{axis_idx}, #{ref_direction_idx});
{location}{axis}{ref_direction}"
                ))
            }
            Self::ConicalSurface(processor) => {
                let revo = processor.entity();
                let transform = processor.transform();
                let line = revo.entity_curve();
                let p = line.0;
                let v = line.1 - p;

                let radius = FloatDisplay(p.x);
                let semi_angle = FloatDisplay(f64::atan(v.x));

                let position_idx = idx + 1;
                let location_idx = idx + 2;
                let axis_idx = idx + 3;
                let ref_direction_idx = idx + 4;

                let location = StepDisplay::new(transform[3].to_point(), location_idx);
                let raw_axis = VectorAsDirection(transform[2].truncate());
                let axis = StepDisplay::new(raw_axis, axis_idx);
                let raw_ref_direction = VectorAsDirection(transform[0].truncate());
                let ref_direction = StepDisplay::new(raw_ref_direction, ref_direction_idx);

                f.write_fmt(format_args!(
                    "#{idx} = CONICAL_SURFACE('', #{position_idx}, {radius}, {semi_angle});
#{position_idx} = AXIS2_PLACEMENT_3D('', #{location_idx}, #{axis_idx}, #{ref_direction_idx});
{location}{axis}{ref_direction}"
                ))
            }
        }
    }
}

/// Step length of a single [`GeometricSetSelect`] element.
fn set_select_step_length(elem: &GeometricSetSelect) -> usize {
    match elem {
        GeometricSetSelect::Curve(c) => Curve3D::try_from(c.as_ref())
            .map(|c3d| c3d.step_length())
            .unwrap_or(0),
        // Point3 has ConstStepLength = 1.
        GeometricSetSelect::Point(_) => 1,
    }
}

impl save::StepLength for GeometricCurveSet {
    fn step_length(&self) -> usize {
        // 1 for the GEOMETRIC_CURVE_SET entity + sum of element lengths.
        1 + self
            .elements
            .iter()
            .map(set_select_step_length)
            .sum::<usize>()
    }
}

impl save::StepFormat for GeometricCurveSet {
    fn fmt(&self, idx: usize, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Collect element start indices.
        let mut cursor = idx + 1;
        let element_indices: Vec<usize> = self
            .elements
            .iter()
            .filter_map(|e| {
                let len = set_select_step_length(e);
                if len == 0 {
                    return None;
                }
                let this = cursor;
                cursor += len;
                Some(this)
            })
            .collect();

        // Write the GEOMETRIC_CURVE_SET entity.
        f.write_fmt(format_args!(
            "#{idx} = GEOMETRIC_CURVE_SET('{}', {});\n",
            self.label,
            IndexSliceDisplay(element_indices.into_iter()),
        ))?;

        // Write each element.
        let mut cursor = idx + 1;
        for elem in &self.elements {
            match elem {
                GeometricSetSelect::Curve(c) => {
                    if let Ok(c3d) = Curve3D::try_from(c.as_ref()) {
                        save::StepFormat::fmt(&c3d, cursor, f)?;
                        cursor += c3d.step_length();
                    }
                }
                GeometricSetSelect::Point(p) => {
                    let pt = Point3::from(p.as_ref());
                    save::StepFormat::fmt(&pt, cursor, f)?;
                    cursor += 1;
                }
            }
        }
        Ok(())
    }
}
