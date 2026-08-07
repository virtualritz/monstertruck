//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

/// Tests if `connect` will properly connect two edges which share only one face.
#[test]
fn test_tnurcc_edge_connect_single_shared_face() {
    use TnurccConnection::*;
    use TnurccFaceSide::*;
    // Primary control points
    let primary_origin = Arc::new(RwLock::new(TnurccControlPoint::new(0, (0.0, 0.0, 0.0))));
    let prmiary_dest = Arc::new(RwLock::new(TnurccControlPoint::new(1, (0.0, 1.0, 0.0))));

    // Primary edge
    let primary_edge = TnurccEdge::new(
        0,
        1.0,
        Arc::clone(&primary_origin),
        Arc::clone(&prmiary_dest),
    );

    // Faces for orientation
    let left_face = Arc::new(RwLock::new(TnurccFace {
        index: 0,
        edge: Some(Arc::clone(&primary_edge)),
        corners: [const { None }; 4],
    }));
    let right_face = Arc::new(RwLock::new(TnurccFace {
        index: 1,
        edge: Some(Arc::clone(&primary_edge)),
        corners: [const { None }; 4],
    }));

    // Connection of faces
    primary_edge.write().face_left = Some(Arc::clone(&left_face));
    primary_edge.write().face_right = Some(Arc::clone(&right_face));

    // The four points which the secondary may connect to. Each on is located in a corner, bl = bottom left and so on.
    // The secondary vector will connect to one of these and one of the primary control points
    let secondary_bl = Arc::new(RwLock::new(TnurccControlPoint::new(2, (-1.0, 0.0, 0.0))));
    let secondary_br = Arc::new(RwLock::new(TnurccControlPoint::new(3, (1.0, 0.0, 0.0))));
    let secondary_tr = Arc::new(RwLock::new(TnurccControlPoint::new(4, (1.0, 1.0, 0.0))));
    let secondary_tl = Arc::new(RwLock::new(TnurccControlPoint::new(5, (-1.0, 1.0, 0.0))));

    // All possible valid configurations
    let test_parameters = vec![
        (
            Arc::clone(&secondary_bl),   // Secondary origin
            Arc::clone(&primary_origin), // Secondary dest
            Left,                        // Common face
            Left,                        // Secondary edge common face side
            LeftCw,                      // Primary edge connection side
            LeftAcw,                     // Secondary edge connectioin side
        ),
        (
            Arc::clone(&prmiary_dest),
            Arc::clone(&secondary_tl),
            Left,
            Left,
            LeftAcw,
            LeftCw,
        ),
        (
            Arc::clone(&secondary_br),
            Arc::clone(&primary_origin),
            Right,
            Right,
            RightAcw,
            RightCw,
        ),
        (
            Arc::clone(&prmiary_dest),
            Arc::clone(&secondary_tr),
            Right,
            Right,
            RightCw,
            RightAcw,
        ),
        (
            Arc::clone(&primary_origin),
            Arc::clone(&secondary_bl),
            Left,
            Right,
            LeftCw,
            RightAcw,
        ),
        (
            Arc::clone(&secondary_tl),
            Arc::clone(&prmiary_dest),
            Left,
            Right,
            LeftAcw,
            RightCw,
        ),
        (
            Arc::clone(&primary_origin),
            Arc::clone(&secondary_br),
            Right,
            Left,
            RightAcw,
            LeftCw,
        ),
        (
            Arc::clone(&secondary_tr),
            Arc::clone(&prmiary_dest),
            Right,
            Left,
            RightCw,
            LeftAcw,
        ),
    ];

    for (org, dst, cmn_f, cmn_f_side, p_con_side, s_con_side) in test_parameters {
        let secondary_edge = TnurccEdge::new(1, 1.0, Arc::clone(&org), Arc::clone(&dst));
        let common_face = Arc::clone(if cmn_f == Left {
            &left_face
        } else {
            &right_face
        });
        match cmn_f_side {
            Left => secondary_edge.write().face_left = Some(common_face),
            Right => secondary_edge.write().face_right = Some(common_face),
        };

        let con_res = TnurccEdge::connect(Arc::clone(&primary_edge), Arc::clone(&secondary_edge));
        assert!(
            con_res.is_ok(),
            "Connection between {:?}->{:?} and {:?}->{:?} failed with error: {}.",
            primary_edge.read().origin.read().point,
            primary_edge.read().dest.read().point,
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point,
            con_res.err().unwrap()
        );

        // Check if the primary edge is connected to the secondary
        let primary_con_orientaion = primary_edge
            .read()
            .connection_orientation(Arc::clone(&secondary_edge));
        assert_eq!(
            primary_con_orientaion.len(),
            1,
            "Primary edge is not connected to secondary the correct number of times \
            for secondary {:?}->{:?}.",
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point
        );
        let primary_con_orientaion = primary_con_orientaion[0];

        // Check if the connection orientation is correct
        assert_eq!(
            p_con_side,
            primary_con_orientaion,
            "Primary edge is not correctly connected to secondary edge for \
            secondary edge {:?}->{:?}.",
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point
        );

        // Check if the secondary edge is connected to the primary
        let secondary_con_orientaion = secondary_edge
            .read()
            .connection_orientation(Arc::clone(&primary_edge));
        assert_eq!(
            secondary_con_orientaion.len(),
            1,
            "Secondary edge is not connected to primary the correct number of times \
            for secondary {:?}->{:?}.",
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point
        );
        let secondary_con_orientaion = secondary_con_orientaion[0];

        // Check if the connection orientation is correct
        assert_eq!(
            s_con_side,
            secondary_con_orientaion,
            "Secondary edge is not correctly connected to primary edge for \
            secondary edge {:?}->{:?}.",
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point
        );

        // Reset the primary edge.
        primary_edge
            .write()
            .set_connection(Arc::clone(&primary_edge), p_con_side);
    }
}

/// Tests if `connect` will properly connect two edges which are "inline" with each other, that is, they share two faces.
#[test]
fn test_tnurcc_edge_connect_double_shared_face() {
    use TnurccConnection::*;
    use TnurccFaceSide::*;
    // Primary control points
    let primary_origin = Arc::new(RwLock::new(TnurccControlPoint::new(0, (0.0, 0.0, 0.0))));
    let prmiary_dest = Arc::new(RwLock::new(TnurccControlPoint::new(1, (0.0, 1.0, 0.0))));

    // The primary edge
    let primary_edge = TnurccEdge::new(
        0,
        1.0,
        Arc::clone(&primary_origin),
        Arc::clone(&prmiary_dest),
    );

    // Faces
    let left_face = Arc::new(RwLock::new(TnurccFace {
        index: 0,
        edge: Some(Arc::clone(&primary_edge)),
        corners: [const { None }; 4],
    }));
    let right_face = Arc::new(RwLock::new(TnurccFace {
        index: 1,
        edge: Some(Arc::clone(&primary_edge)),
        corners: [const { None }; 4],
    }));

    // Set the faces of the primary
    primary_edge.write().face_left = Some(Arc::clone(&left_face));
    primary_edge.write().face_right = Some(Arc::clone(&right_face));

    // Two points which are on either side of the primary
    let secondary_top = Arc::new(RwLock::new(TnurccControlPoint::new(2, (0.0, 2.0, 0.0))));
    let secondary_bottom = Arc::new(RwLock::new(TnurccControlPoint::new(3, (0.0, -1.0, 0.0))));

    // The various possible valid configurations for connecting the two points in this way.
    let test_parameters = vec![
        (
            Arc::clone(&secondary_top), // Secondary origin
            Arc::clone(&prmiary_dest),  // Secondary dest
            Right,                      // left_face side
            [LeftAcw, RightCw],         // Secondary to primary connections
            [LeftAcw, RightCw],         // Primary to secondary connections
        ),
        (
            Arc::clone(&prmiary_dest),
            Arc::clone(&secondary_top),
            Left,
            [LeftCw, RightAcw],
            [LeftAcw, RightCw],
        ),
        (
            Arc::clone(&secondary_bottom),
            Arc::clone(&primary_origin),
            Left,
            [LeftAcw, RightCw],
            [LeftCw, RightAcw],
        ),
        (
            Arc::clone(&primary_origin),
            Arc::clone(&secondary_bottom),
            Right,
            [LeftCw, RightAcw],
            [LeftCw, RightAcw],
        ),
    ];

    for (s_org, s_dest, left_face_side, s_con_sides, p_con_sides) in test_parameters {
        // Construct the secondary edge according to the provided parameters
        let secondary_edge = TnurccEdge::new(1, 1.0, Arc::clone(&s_org), Arc::clone(&s_dest));

        // Face orientation varries
        if left_face_side == Left {
            secondary_edge.write().face_left = Some(Arc::clone(&left_face));
            secondary_edge.write().face_right = Some(Arc::clone(&right_face));
        } else {
            secondary_edge.write().face_left = Some(Arc::clone(&right_face));
            secondary_edge.write().face_right = Some(Arc::clone(&left_face));
        }

        // Attempt to connect
        let con_res = TnurccEdge::connect(Arc::clone(&primary_edge), Arc::clone(&secondary_edge));
        assert!(
            con_res.is_ok(),
            "Connection between {:?}->{:?} and {:?}->{:?} failed with error: {}.",
            primary_edge.read().origin.read().point,
            primary_edge.read().dest.read().point,
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point,
            con_res.err().unwrap()
        );

        // Check if the primary edge is connected to the secondary
        let primary_con_orientaion = primary_edge
            .read()
            .connection_orientation(Arc::clone(&secondary_edge));
        assert_eq!(
            primary_con_orientaion.len(),
            2,
            "Primary edge is not connected to secondary the correct number of times \
            for secondary {:?}->{:?}.",
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point
        );

        // Check if the connection orientation is correct
        for expect in p_con_sides.iter() {
            assert!(
                primary_con_orientaion.contains(expect),
                "Primary edge is not correctly connected to secondary edge for \
                secondary edge {:?}->{:?}'s connection {}.",
                secondary_edge.read().origin.read().point,
                secondary_edge.read().dest.read().point,
                *expect
            );
        }

        // Check if the secondary edge is connected to the primary
        let secondary_con_orientaion = secondary_edge
            .read()
            .connection_orientation(Arc::clone(&primary_edge));
        assert_eq!(
            secondary_con_orientaion.len(),
            2,
            "Secondary edge is not connected to primary the correct number of times \
            for secondary {:?}->{:?}.",
            secondary_edge.read().origin.read().point,
            secondary_edge.read().dest.read().point
        );

        // Check if the connection orientation is correct
        for expect in s_con_sides.iter() {
            assert!(
                secondary_con_orientaion.contains(expect),
                "Secondary edge is not correctly connected to primary edge for \
                secondary edge {:?}->{:?}'s connection {}.",
                secondary_edge.read().origin.read().point,
                secondary_edge.read().dest.read().point,
                *expect
            );
        }

        // Reset the primary edge.
        for i in 0..4 {
            primary_edge
                .write()
                .set_connection(Arc::clone(&primary_edge), TnurccConnection::from_usize(i));
        }
    }
}

/// Tests `split_edge`, checking to make sure that the logic for splitting the edge into two,
/// with a control point between them, is functioning as expected
#[test]
fn test_tnurcc_split_edge() {
    // Control points needed
    let origin = Arc::new(RwLock::new(TnurccControlPoint::new(
        0,
        Point3::from((0.0, 0.0, 0.0)),
    )));
    let dest = Arc::new(RwLock::new(TnurccControlPoint::new(
        1,
        Point3::from((0.0, 5.0, 0.0)),
    )));

    // Edge to be split
    let edge = TnurccEdge::new(0, 2.5, Arc::clone(&origin), Arc::clone(&dest));

    // Faces for connections
    let left_face = Arc::new(RwLock::new(TnurccFace {
        index: 0,
        edge: Some(Arc::clone(&edge)),
        corners: [const { None }; 4],
    }));
    let right_face = Arc::new(RwLock::new(TnurccFace {
        index: 0,
        edge: Some(Arc::clone(&edge)),
        corners: [const { None }; 4],
    }));

    edge.write().face_left = Some(Arc::clone(&left_face));
    edge.write().face_right = Some(Arc::clone(&right_face));

    // Set the incoming edge to an edge it wont be connected to after edge splitting, to test if it gets reasigned correctly
    dest.write().incoming_edge = Some(Arc::clone(&edge));

    // Split the edge
    let middle = TnurccEdge::split_edge(
        Arc::clone(&edge),
        24,
        Point3::from((0.0, 1.0, 0.0)),
        56,
        0.25,
    )
    .expect("Splitting is designed to succeed");

    // Get the new edge, other_left and other_right should be the same
    let other_left = edge.read().connection(TnurccConnection::LeftAcw);
    let other_right = edge.read().connection(TnurccConnection::RightCw);

    // Check that edge is correctly connected to an edge
    assert!(
        std::ptr::eq(other_left.as_ref(), other_right.as_ref()),
        "New edge was not properly connected."
    );
    // Check that the above edge is a new edge
    assert!(
        !std::ptr::eq(other_left.as_ref(), edge.as_ref()),
        "New edge was not properly created or connected."
    );
    // Check that the old edge was correctly modified
    assert!(
        std::ptr::eq(edge.read().dest.as_ref(), middle.as_ref()),
        "Edge's destination is incorrect."
    );
    // Check that the new edge has the correct desitination
    assert!(
        std::ptr::eq(other_left.read().dest.as_ref(), dest.as_ref()),
        "New edge's destination is incorrect."
    );
    // Check that edge's knot interval is correct
    assert!(
        (edge.read().knot_interval - 0.625).so_small(),
        "Edge's knot interval is incorrect."
    );
    // Check that the new edge's knot interval is correct
    assert!(
        (other_left.read().knot_interval - 1.875).so_small(),
        "New edge's knot interval is incorrect."
    );
    // Check the valence of the new point
    assert_eq!(middle.read().valence, 2, "Point valence is incorrect.");
    // Check the index of the new point
    assert_eq!(middle.read().index, 56, "Point index is incorrect.");
    // Check that the new point's incomming edge is an edge that is
    // connected to the new point
    assert!(
        middle
            .read()
            .incoming_edge
            .as_ref()
            .unwrap()
            .read()
            .point_end(Arc::clone(&middle))
            .is_some(),
        "Middle's incomming edge is incorrect."
    );
    // Check that the new edge's index is correct
    assert_eq!(other_left.read().index, 24, "New edge index is incorrect.");

    // Check that the destination point's incoming edge has been correctly set
    assert!(
        std::ptr::eq(
            other_left.as_ref(),
            dest.read()
                .incoming_edge
                .as_ref()
                .expect("Incoming edge for dest should have been set")
                .as_ref()
        ),
        "Incomming edge for destination was not correctly set to new edge."
    );
}

/// Tests if splitting an edge which is connected to other edges correctly mutates the edges it is connected to,
/// so that they no longer refer to an edge they aren't topologically connected to.
#[test]
fn test_tnurcc_split_edge_connected_edges() {
    // Control points to be used in the test. tl and tr are top left and top right respectively
    let origin = Arc::new(RwLock::new(TnurccControlPoint::new(
        0,
        Point3::from((0.0, 0.0, 0.0)),
    )));
    let dest = Arc::new(RwLock::new(TnurccControlPoint::new(
        1,
        Point3::from((0.0, 5.0, 0.0)),
    )));
    let tl = Arc::new(RwLock::new(TnurccControlPoint::new(
        1,
        Point3::from((-1.0, 5.0, 0.0)),
    )));
    let tr = Arc::new(RwLock::new(TnurccControlPoint::new(
        1,
        Point3::from((1.0, 5.0, 0.0)),
    )));

    // Edges to be used. left_edge and right_edge are the edges which will not be split,
    // but must be reconnected to the new edge.
    let left_edge = TnurccEdge::new(0, 2.5, Arc::clone(&tl), Arc::clone(&dest));
    let right_edge = TnurccEdge::new(1, 2.5, Arc::clone(&tr), Arc::clone(&dest));
    let edge = TnurccEdge::new(2, 2.5, Arc::clone(&origin), Arc::clone(&dest));

    // Faces, needed for connecting and reconnecting
    let left_face = Arc::new(RwLock::new(TnurccFace {
        index: 0,
        edge: Some(Arc::clone(&edge)),
        corners: [const { None }; 4],
    }));
    let right_face = Arc::new(RwLock::new(TnurccFace {
        index: 0,
        edge: Some(Arc::clone(&edge)),
        corners: [const { None }; 4],
    }));

    // Set the faces of the edges and connect them together
    edge.write().face_left = Some(Arc::clone(&left_face));
    edge.write().face_right = Some(Arc::clone(&right_face));

    left_edge.write().face_right = Some(Arc::clone(&left_face));
    right_edge.write().face_left = Some(Arc::clone(&right_face));

    TnurccEdge::connect(Arc::clone(&edge), Arc::clone(&right_edge))
        .expect("Connection should be topologically consistent");
    TnurccEdge::connect(Arc::clone(&edge), Arc::clone(&left_edge))
        .expect("Connection should be topologically consistent");

    // Split the edge (parameters are the same from the previous test, but will not be checked)
    let _middle = TnurccEdge::split_edge(
        Arc::clone(&edge),
        24,
        Point3::from((0.0, 1.0, 0.0)),
        56,
        0.25,
    );

    // Get the new edge
    let new_edge = edge.read().connection(TnurccConnection::LeftAcw);

    // Test that left_edge was reconnected to the new edge
    assert!(
        std::ptr::eq(
            left_edge
                .read()
                .connection(TnurccConnection::RightCw)
                .as_ref(),
            new_edge.as_ref()
        ),
        "Edge's left anti-clockwise connection was not correctly redirected."
    );
    // Test that right_edge was reconnected to the new edge
    assert!(
        std::ptr::eq(
            right_edge
                .read()
                .connection(TnurccConnection::LeftAcw)
                .as_ref(),
            new_edge.as_ref()
        ),
        "Edge's right clockwise connection was not correctly redirected."
    );
    // Test that the new edge was connected to right_edge
    assert!(
        std::ptr::eq(
            right_edge.as_ref(),
            new_edge
                .read()
                .connection(TnurccConnection::RightCw)
                .as_ref()
        ),
        "Edge's right clockwise connection was not correctly transfered."
    );
    // Test that the new edge was connected to left_edge
    assert!(
        std::ptr::eq(
            left_edge.as_ref(),
            new_edge
                .read()
                .connection(TnurccConnection::LeftAcw)
                .as_ref()
        ),
        "Edge's left anti-clockwise connection was not correctly transfered."
    );
}
