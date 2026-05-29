from __future__ import annotations

import os
import re
import textwrap
from dataclasses import dataclass
from datetime import date
from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    PageBreak,
    PageTemplate,
    Paragraph,
    Preformatted,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "VexAI1_Code_Reference.pdf"


def rel(path: str) -> Path:
    return ROOT / path


def read_lines(path: str) -> list[str]:
    return rel(path).read_text(encoding="utf-8", errors="replace").splitlines()


def line_no(path: str, pattern: str, start: int = 1) -> int:
    regex = re.compile(pattern)
    for idx, line in enumerate(read_lines(path), start=1):
        if idx < start:
            continue
        if regex.search(line):
            return idx
    raise ValueError(f"Pattern not found in {path}: {pattern}")


def snippet(path: str, pattern: str, count: int, start_offset: int = 0, wrap: int = 94) -> str:
    lines = read_lines(path)
    start = line_no(path, pattern) + start_offset
    end = min(len(lines), start + count - 1)
    out: list[str] = []
    for no in range(start, end + 1):
        raw = lines[no - 1].rstrip()
        prefix = f"{no:4d}: "
        if len(raw) <= wrap:
            out.append(prefix + raw)
            continue
        chunks = textwrap.wrap(
            raw,
            width=wrap,
            replace_whitespace=False,
            drop_whitespace=False,
            subsequent_indent="      ",
        )
        if not chunks:
            out.append(prefix)
        else:
            out.append(prefix + chunks[0])
            for chunk in chunks[1:]:
                out.append("      " + chunk)
    return "\n".join(out)


def para(text: str, style: ParagraphStyle):
    return Paragraph(escape(text), style)


def code_block(text: str, styles):
    return Preformatted(text, styles["RefCode"])


@dataclass
class FunctionNote:
    name: str
    file: str
    purpose: str
    details: str


class CodeReferenceDoc(BaseDocTemplate):
    def __init__(self, filename: str):
        super().__init__(
            filename,
            pagesize=letter,
            leftMargin=0.75 * inch,
            rightMargin=0.75 * inch,
            topMargin=0.78 * inch,
            bottomMargin=0.7 * inch,
        )
        frame = Frame(
            self.leftMargin,
            self.bottomMargin,
            self.width,
            self.height,
            id="normal",
        )
        self.addPageTemplates(
            [
                PageTemplate(
                    id="pages",
                    frames=[frame],
                    onPage=self._page_header_footer,
                )
            ]
        )

    def _page_header_footer(self, canvas, doc):
        canvas.saveState()
        canvas.setFont("Helvetica", 8)
        canvas.setFillColor(colors.HexColor("#56616F"))
        canvas.drawString(0.75 * inch, 10.45 * inch, "VexAI1 Code Reference")
        canvas.drawRightString(7.75 * inch, 10.45 * inch, f"Page {doc.page}")
        canvas.setStrokeColor(colors.HexColor("#D8DEE8"))
        canvas.line(0.75 * inch, 10.35 * inch, 7.75 * inch, 10.35 * inch)
        canvas.restoreState()


def build_styles():
    styles = getSampleStyleSheet()
    styles.add(
        ParagraphStyle(
            name="TitleBlue",
            parent=styles["Title"],
            fontName="Helvetica-Bold",
            fontSize=22,
            leading=26,
            textColor=colors.HexColor("#0B2545"),
            alignment=TA_CENTER,
            spaceAfter=18,
        )
    )
    styles.add(
        ParagraphStyle(
            name="RefSubtitle",
            parent=styles["Normal"],
            fontName="Helvetica",
            fontSize=10,
            leading=14,
            textColor=colors.HexColor("#56616F"),
            alignment=TA_CENTER,
            spaceAfter=20,
        )
    )
    styles.add(
        ParagraphStyle(
            name="H1",
            parent=styles["Heading1"],
            fontName="Helvetica-Bold",
            fontSize=16,
            leading=20,
            textColor=colors.HexColor("#2E74B5"),
            spaceBefore=18,
            spaceAfter=10,
            keepWithNext=True,
        )
    )
    styles.add(
        ParagraphStyle(
            name="H2",
            parent=styles["Heading2"],
            fontName="Helvetica-Bold",
            fontSize=13,
            leading=16,
            textColor=colors.HexColor("#2E74B5"),
            spaceBefore=14,
            spaceAfter=7,
            keepWithNext=True,
        )
    )
    styles.add(
        ParagraphStyle(
            name="H3",
            parent=styles["Heading3"],
            fontName="Helvetica-Bold",
            fontSize=11,
            leading=14,
            textColor=colors.HexColor("#1F4D78"),
            spaceBefore=10,
            spaceAfter=5,
            keepWithNext=True,
        )
    )
    styles.add(
        ParagraphStyle(
            name="Body",
            parent=styles["BodyText"],
            fontName="Helvetica",
            fontSize=9.3,
            leading=11.8,
            spaceAfter=6,
            alignment=TA_LEFT,
        )
    )
    styles.add(
        ParagraphStyle(
            name="RefSmall",
            parent=styles["BodyText"],
            fontName="Helvetica",
            fontSize=8.1,
            leading=10,
            textColor=colors.HexColor("#27313D"),
            spaceAfter=4,
        )
    )
    styles.add(
        ParagraphStyle(
            name="RefCode",
            fontName="Courier",
            fontSize=6.7,
            leading=8.1,
            leftIndent=0,
            rightIndent=0,
            backColor=colors.HexColor("#F4F6F9"),
            borderColor=colors.HexColor("#D8DEE8"),
            borderWidth=0.4,
            borderPadding=5,
            spaceBefore=4,
            spaceAfter=8,
        )
    )
    styles.add(
        ParagraphStyle(
            name="RefBullet",
            parent=styles["Body"],
            leftIndent=14,
            firstLineIndent=-8,
            bulletIndent=0,
            spaceAfter=3,
        )
    )
    styles.add(
        ParagraphStyle(
            name="RefTableCell",
            parent=styles["RefSmall"],
            fontSize=7.9,
            leading=9.5,
            spaceAfter=0,
        )
    )
    return styles


def h1(story, text, styles):
    story.append(Paragraph(escape(text), styles["H1"]))


def h2(story, text, styles):
    story.append(Paragraph(escape(text), styles["H2"]))


def h3(story, text, styles):
    story.append(Paragraph(escape(text), styles["H3"]))


def p(story, text, styles):
    story.append(Paragraph(escape(text), styles["Body"]))


def bullet(story, text, styles):
    story.append(Paragraph(escape(text), styles["RefBullet"], bulletText="-"))


def snippet_flow(story, title, path, pattern, count, styles, start_offset=0):
    h3(story, title, styles)
    story.append(code_block(snippet(path, pattern, count, start_offset=start_offset), styles))


def note_table(story, rows: list[FunctionNote], styles):
    data = [
        [
            Paragraph("<b>Function / item</b>", styles["RefTableCell"]),
            Paragraph("<b>File</b>", styles["RefTableCell"]),
            Paragraph("<b>How it works</b>", styles["RefTableCell"]),
        ]
    ]
    for row in rows:
        detail = f"<b>{escape(row.purpose)}</b><br/>{escape(row.details)}"
        data.append(
            [
                Paragraph(f"<font name='Courier'>{escape(row.name)}</font>", styles["RefTableCell"]),
                Paragraph(escape(row.file), styles["RefTableCell"]),
                Paragraph(detail, styles["RefTableCell"]),
            ]
        )
    table = Table(data, colWidths=[1.65 * inch, 1.45 * inch, 4.15 * inch], repeatRows=1)
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#E8EEF5")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.HexColor("#0B2545")),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#C9D3E1")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 5),
                ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    story.append(table)
    story.append(Spacer(1, 8))


def overview(story, styles):
    h1(story, "1. System Overview", styles)
    p(
        story,
        "This project is a command-based VEX V5 autonomous program. The main loop repeatedly updates odometry, runs a scheduler, applies drivetrain and intake motor powers, and prints diagnostics to the Brain. Higher-level robot behavior is built from Command objects and command groups.",
        styles,
    )
    bullet(story, "Hardware definitions live in RobotConfig.cpp / RobotConfig.h.", styles)
    bullet(story, "Command scheduling lives in Scheduler.h plus the command group classes.", styles)
    bullet(story, "Autonomous order and tuning constants live mostly in Routines.cpp.", styles)
    bullet(story, "Vision data comes from the Jetson serial parser, then FindBlockCommand and TrackBlockCommand choose allowed blocks.", styles)
    bullet(story, "Field movement is handled by drive commands, GPS pose tracking, inertial turning, and wall alignment.", styles)
    bullet(story, "Intake behavior includes color sorting, block counting, unjamming, and scoring.", styles)
    snippet_flow(story, "Main control loop", "src/main.cpp", r"int main\(\)", 26, styles)


def command_framework(story, styles):
    h1(story, "2. Command Framework", styles)
    p(
        story,
        "Every action implements the Command interface. initialize() sets command state, execute() runs one 10 ms loop slice, isFinished() tells the scheduler when to end it, and end() stops or cleans up.",
        styles,
    )
    snippet_flow(story, "Base command interface", "include/Command.h", r"class Command", 8, styles)
    snippet_flow(story, "Scheduler run loop", "include/Scheduler.h", r"void run\(\)", 32, styles)
    note_table(
        story,
        [
            FunctionNote("Scheduler::schedule", "include/Scheduler.h", "Queues a command.", "Adds a Command pointer and marks it uninitialized so initialize() runs on the next scheduler cycle."),
            FunctionNote("Scheduler::run", "include/Scheduler.h", "Runs every scheduled command.", "Initializes unstarted commands, calls execute(), then calls end() and removes commands whose isFinished() returns true."),
            FunctionNote("SequentialCommandGroup::execute", "src/SequentialCommandGroup.cpp", "Runs one command at a time.", "Initializes the current command, executes it until finished, ends it, then advances to the next command."),
            FunctionNote("ParallelCommandGroup::execute", "src/ParallelCommandGroup.cpp", "Runs commands together.", "Executes every child each loop and finishes only when all children are finished."),
            FunctionNote("ParallelDeadlineGroup::execute", "src/ParallelDeadlineGroup.cpp", "Runs helpers until a deadline ends.", "The deadline command controls group completion; helper commands run in parallel and are ended when the group ends."),
            FunctionNote("RaceCommandGroup::execute", "src/RaceCommandGroup.cpp", "First command to finish wins.", "Executes every child and finishes the group as soon as any child reports done."),
            FunctionNote("ConditionalCommandGroup::initialize", "src/ConditionalCommandGroup.cpp", "Chooses one branch.", "Evaluates a boolean function and initializes either the true command or false command."),
            FunctionNote("RepeatForeverCommandGroup::execute", "src/RepeatForeverCommandGroup.cpp", "Loops a command group forever.", "Runs the wrapped command; when it finishes, calls end() and initialize() to restart it."),
        ],
        styles,
    )


def hardware_pose(story, styles):
    h1(story, "3. Hardware, Pose, and Debug Status", styles)
    p(
        story,
        "RobotConfig.cpp owns the VEX hardware objects. Drivetrain stores the requested motor powers and applies them later in the main loop. PositionTracking reads GPS, applies the physical GPS sensor offset, and exposes robot-center field coordinates to commands.",
        styles,
    )
    snippet_flow(story, "GPS sensor offset correction", "src/PositionTracking.cpp", r"float PositionTracking::get_raw_x", 27, styles)
    note_table(
        story,
        [
            FunctionNote("Drivetrain::set_odom_pose", "src/Drivetrain.cpp", "Sets x/y/theta pose.", "Stores pose and calls set_heading_degrees() so the inertial heading is aligned to the pose heading."),
            FunctionNote("Drivetrain::set_heading_degrees", "src/Drivetrain.cpp", "Safely writes inertial heading.", "Rejects non-finite headings, normalizes to 0..359.999, then calls drivetrain_inertial.setHeading()."),
            FunctionNote("Drivetrain::set_drive_power", "src/Drivetrain.cpp", "Stores requested power.", "Does not spin motors immediately; main() calls apply_motor_power() once per loop."),
            FunctionNote("Drivetrain::apply_motor_power", "src/Drivetrain.cpp", "Applies drivetrain output.", "Spins the left and right motor groups using the latest stored percentages."),
            FunctionNote("PositionTracking::get_raw_x/y", "src/PositionTracking.cpp", "Returns robot-center GPS pose.", "Reads the GPS sensor position and subtracts the rotated offset: 4 inches right, 7 inches behind the robot center."),
            FunctionNote("PositionTracking::update_raw_pose", "src/PositionTracking.cpp", "Refreshes cached pose.", "Stores corrected x/y plus normalized GPS heading in gpsPose."),
            FunctionNote("CommandStatus setters", "src/CommandStatus.cpp", "Stores debug display values.", "Holds current command label and last wall alignment distance for Brain screen output."),
            FunctionNote("JetsonSerial::print_block_pos_on_screen", "src/JetsonSerial.cpp", "Brain diagnostic display.", "Prints selected block, red count, vision errors, wall distance, GPS, IMU, and current command status."),
        ],
        styles,
    )
    snippet_flow(story, "Brain display rows", "src/JetsonSerial.cpp", r"void JetsonSerial::print_block_pos_on_screen", 35, styles)


def jetson_vision(story, styles):
    h1(story, "4. Jetson Vision Data Path", styles)
    p(
        story,
        "The Jetson sends either 0 or a semicolon-separated list of block coordinates. JetsonSerial parses that into arrays, then raw find/track functions select a block. The higher-level find and track commands now filter those raw candidates through field avoid zones before accepting a block.",
        styles,
    )
    snippet_flow(story, "Multi-block serial parser", "src/JetsonSerial.cpp", r"void JetsonSerial::update_block_pose", 70, styles)
    snippet_flow(story, "Generic closest-block helper", "src/JetsonSerial.cpp", r"bool JetsonSerial::select_block_closest_to", 26, styles)
    note_table(
        story,
        [
            FunctionNote("clear_blocks", "src/JetsonSerial.cpp", "Resets vision frame state.", "Clears selected position and every stored block coordinate before parsing a new serial message."),
            FunctionNote("parse_xy_pair", "src/JetsonSerial.cpp", "Parses one coordinate pair.", "Splits a token at a comma and converts x/y text to integers."),
            FunctionNote("select_block_index", "src/JetsonSerial.cpp", "Selects an array item.", "Copies block_x_positions[index] and block_y_positions[index] into the active selected block."),
            FunctionNote("select_block_closest_to", "src/JetsonSerial.cpp", "Generic unfiltered chooser.", "Finds the block closest to a target pixel by Manhattan distance. This does not know about avoid zones."),
            FunctionNote("update_block_pose", "src/JetsonSerial.cpp", "Reads one serial frame.", "Supports old x,y format and new count;x,y;x,y format; after parsing it selects index 0 by default."),
            FunctionNote("find_block_raw_step", "src/JetsonSerial.cpp", "Raw stable-block detector.", "Checks whether selected block movement is small for enough sequential frames. The higher-level FindBlockCommand replaces selection with avoid-zone-aware selection."),
            FunctionNote("track_block_raw_step", "src/JetsonSerial.cpp", "Raw tracker.", "Maintains a target lock using jump thresholds and lost-frame tolerance. The higher-level commands now use allowed-only tracking helpers instead of this generic method when avoid zones matter."),
            FunctionNote("getXError / getYError", "src/JetsonSerial.cpp", "Vision error accessors.", "Return the x and y errors stored in trackBlockRawVar for PID control."),
        ],
        styles,
    )


def field_avoid_and_find(story, styles):
    h1(story, "5. Field Avoid Zones and FindBlockCommand", styles)
    p(
        story,
        "FindBlockCommand is responsible for scanning for a legal block, centering on it, and rejecting blocks whose estimated field position falls inside an avoid zone. Normal search is now rotation in place to the right. The circular arc unstuck sequence only starts if the robot is supposed to rotate but heading does not change.",
        styles,
    )
    snippet_flow(story, "Allowed block selection", "include/FindBlockCommand.h", r"bool selectAllowedBlockClosestTo", 39, styles)
    snippet_flow(story, "Allowed-only tracking while centering", "include/FindBlockCommand.h", r"bool trackAllowedBlockRawStep", 74, styles)
    snippet_flow(story, "Search movement and stuck detection", "include/FindBlockCommand.h", r"void applySearchMovement", 78, styles)
    snippet_flow(story, "FindBlockCommand execute state machine", "include/FindBlockCommand.h", r"void execute\(\) override", 140, styles)
    note_table(
        story,
        [
            FunctionNote("estimateBlockDistanceFromPixelY", "include/FindBlockCommand.h", "Approximates distance from pixel y.", "Uses a polynomial mapping from image y to estimated inches from the robot."),
            FunctionNote("estimateBlockFieldPosition", "include/FindBlockCommand.h", "Projects a block onto the field.", "Uses robot GPS x/y/heading, camera center error, and horizontal FOV to estimate block field coordinates."),
            FunctionNote("pointInsideAvoidZone", "include/FindBlockCommand.h", "Rectangle test.", "Normalizes min/max x/y and returns true if a field point is inside an enabled zone."),
            FunctionNote("blockIsInsideAnyAvoidZone", "include/FindBlockCommand.h", "Checks all 9 zones.", "Returns true if the estimated block field position is in any configured avoid rectangle."),
            FunctionNote("candidateBlockIsInsideAvoidZone", "include/FindBlockCommand.h", "Pixel candidate filter.", "Estimates the candidate field location and calls blockIsInsideAnyAvoidZone()."),
            FunctionNote("selectAllowedBlockClosestTo", "include/FindBlockCommand.h", "Legal block chooser.", "Scans every visible block, skips rejected zones, and chooses the allowed block closest to the target pixel. If all blocks are rejected, it clears the selected block."),
            FunctionNote("findAllowedBlockRawStep", "include/FindBlockCommand.h", "Stable legal-block find step.", "Updates one Jetson frame, selects only legal blocks, checks stability across sequential frames, and returns true once the block is stable enough."),
            FunctionNote("trackAllowedBlockRawStep", "include/FindBlockCommand.h", "Legal-block tracker.", "During centering, it keeps choosing from allowed blocks only, so a rejected block cannot steal the target lock."),
            FunctionNote("applySearchMovement", "include/FindBlockCommand.h", "Search or unstuck motion.", "If not unstucking, rotates in place right. If unstucking, runs forward/right, forward/left, back/right, back/left arcs."),
            FunctionNote("updateStuckDetection", "include/FindBlockCommand.h", "Starts search unstuck only when needed.", "While normally searching, it checks heading change only. If heading is not changing, it starts the arc sequence."),
            FunctionNote("centeringMadeProgress", "include/FindBlockCommand.h", "Detects centering stall.", "While rotating to center a block, expects either heading movement or x-error improvement."),
            FunctionNote("execute", "include/FindBlockCommand.h", "Main state machine.", "SEARCHING_FOR_BLOCK rotates and finds a stable allowed block. CENTERING_BLOCK tracks allowed blocks, centers by x error, and finishes when centered."),
        ],
        styles,
    )


def tracking_section(story, styles):
    h1(story, "6. TrackBlockCommand", styles)
    p(
        story,
        "TrackBlockCommand drives toward a selected block using x error for turning and y error for forward motion. Its allowed-only tracking step mirrors the find command: it filters rejected field zones first, then picks the closest acceptable block to the last tracked position.",
        styles,
    )
    snippet_flow(story, "Track allowed-only selector", "include/TrackBlockCommand.h", r"bool selectAllowedBlockClosestTo", 39, styles)
    snippet_flow(story, "TrackBlockCommand execute loop", "include/TrackBlockCommand.h", r"void execute\(\) override", 120, styles)
    note_table(
        story,
        [
            FunctionNote("failStuck", "include/TrackBlockCommand.h", "Ends with stuck failure.", "Stops drive power, sets result to TRACK_BLOCK_STUCK_FAILED, and finishes."),
            FunctionNote("estimateBlockFieldPosition", "include/TrackBlockCommand.h", "Same projection as find.", "Uses GPS pose plus camera bearing to estimate where the block is on the field."),
            FunctionNote("trackAllowedBlockRawStep", "include/TrackBlockCommand.h", "Avoid-zone-aware tracking.", "Updates Jetson frame, picks only accepted blocks, applies jump thresholds, and maintains lost-frame behavior."),
            FunctionNote("robotMadeProgressWhileTracking", "include/TrackBlockCommand.h", "Progress detector.", "Accepts progress from encoder movement, heading movement, or enough y-error change."),
            FunctionNote("runUnstuckState", "include/TrackBlockCommand.h", "Circular arc escape.", "Runs forward/right, forward/left, back/right, back/left, then returns to normal tracking."),
            FunctionNote("execute", "include/TrackBlockCommand.h", "PID block approach.", "Tracks allowed block, rejects avoid-zone targets, calculates angular and linear PID, starts unstuck if y progress stalls, and succeeds when x/y are within thresholds."),
            FunctionNote("wasSuccessful / wasAvoided / lostTracking", "include/TrackBlockCommand.h", "Result helpers.", "Expose the result to Routines.cpp so the robot can decide whether to grab or skip."),
        ],
        styles,
    )


def movement_section(story, styles):
    h1(story, "7. Movement Commands", styles)
    p(
        story,
        "Movement commands share the same pattern: initialize local error/PID state, repeatedly read sensors, compute PID output, clamp speeds, optionally run unstuck logic, and finish on settle time or timeout.",
        styles,
    )
    snippet_flow(story, "Drive-to-point target heading and distance", "include/DriveToPointCommand.h", r"void updatePositionAndTarget", 23, styles)
    snippet_flow(story, "Drive-to-point execute state machine", "include/DriveToPointCommand.h", r"void execute\(\) override", 95, styles)
    snippet_flow(story, "Generic circular arc helper", "include/UnstuckArcHelper.h", r"bool run", 55, styles)
    note_table(
        story,
        [
            FunctionNote("DriveStraightCommand::initialize", "include/DriveStraightCommand.h", "Sets drive-straight state.", "Stores initial encoder position, resets PID state, sets timeout, configures UnstuckArcHelper."),
            FunctionNote("DriveStraightCommand::execute", "include/DriveStraightCommand.h", "Encoder-distance PID.", "Uses wheel encoder distance for linear error and inertial heading for angular correction."),
            FunctionNote("InertialTurnCommand::execute", "include/InertialTurnCommand.h", "Heading PID turn.", "Computes shortest heading error, clamps turn output, uses settle timer and unstuck helper."),
            FunctionNote("DriveToX/YPositionCommand::execute", "include/DriveToXPositionCommand.h / DriveToYPositionCommand.h", "Axis-specific GPS drive.", "Uses GPS x or y error projected onto the requested heading, while continuously correcting heading."),
            FunctionNote("DriveToPointCommand::execute", "include/DriveToPointCommand.h", "Point then drive.", "First turns to face the target, then drives straight toward it. Reverse mode adds 180 degrees to heading and negates linear speed."),
            FunctionNote("DriveToPointUntilX/YCommand::crossedExitLine", "include/DriveToPointUntilXCommand.h / DriveToPointUntilYCommand.h", "Exit-line variant.", "Finishes when the robot crosses a specified x or y boundary instead of waiting to settle at a point."),
            FunctionNote("GrabBlockCommand::execute", "include/GrabBlockCommand.h", "Short encoder drive to collect.", "Sets target heading to current heading at initialize, drives the requested distance, and uses UnstuckArcHelper if progress stalls."),
            FunctionNote("UnstuckArcHelper::shouldStart", "include/UnstuckArcHelper.h", "Generic stuck detector.", "Checks time since last progress and compares encoder, heading, and error progress signals."),
            FunctionNote("UnstuckArcHelper::run", "include/UnstuckArcHelper.h", "Generic escape sequence.", "Runs four arc stages with configured speeds/times and returns true while it owns drivetrain output."),
        ],
        styles,
    )


def wall_and_gps_section(story, styles):
    h1(story, "8. GPS Filtering and Wall Alignment", styles)
    p(
        story,
        "The GPS filter rejects samples that jump too far from a reference sample. Wall alignment uses rear distance sensors and GPS heading to drive backward toward a wall while continuously correcting heading.",
        styles,
    )
    snippet_flow(story, "Filtered GPS sample logic", "include/GetGPSCoordinatesFilteredCommand.h", r"void execute\(\) override", 60, styles)
    snippet_flow(story, "Wall alignment sensor averaging and display", "include/WallAlignmentCommand.h", r"var.left_sensor_detected", 45, styles)
    snippet_flow(story, "Wall alignment drive output", "include/WallAlignmentCommand.h", r"if \(!var.wall_detected\)", 75, styles)
    snippet_flow(story, "Score approach GPS retry gate", "src/Routines.cpp", r"bool robotIsCloseToPoint", 42, styles)
    note_table(
        story,
        [
            FunctionNote("GetGPSCoordinatesFilteredCommand::initialize", "include/GetGPSCoordinatesFilteredCommand.h", "Starts a GPS sample batch.", "Resets sums, reference pose, sample counters, and success flag."),
            FunctionNote("GetGPSCoordinatesFilteredCommand::execute", "include/GetGPSCoordinatesFilteredCommand.h", "Filters GPS data.", "Collects samples at intervals, accepts samples near the reference, averages accepted x/y and heading delta, and writes filtered pose only if enough samples pass."),
            FunctionNote("wasSuccessful", "include/GetGPSCoordinatesFilteredCommand.h", "Reports filter result.", "Used by SetDrivetrainPoseFromGPSCommand and score retry checks to avoid trusting stale GPS data."),
            FunctionNote("SetDrivetrainPoseFromGPSCommand::initialize", "src/Routines.cpp", "Applies filtered pose.", "Skips if filtering failed, rejects non-finite pose values, normalizes heading, then updates drivetrain odom pose."),
            FunctionNote("WallAlignmentCommand::execute", "include/WallAlignmentCommand.h", "Distance and heading alignment.", "Reads both rear distance sensors, uses their average if both see a wall, computes corrected distance by cosine of heading error, then combines linear and angular PID."),
            FunctionNote("WallAlignmentCommand no-wall behavior", "include/WallAlignmentCommand.h", "Drives backward if no wall is seen.", "When neither sensor detects an object, it commands max linear speed before rear-sensor inversion, causing the robot to back toward the wall."),
            FunctionNote("setLastWallAlignmentDistance", "src/CommandStatus.cpp", "Stores final wall reading.", "WallAlignmentCommand updates this whenever either rear sensor sees the wall; Brain row 7 keeps the last reading after the command ends."),
        ],
        styles,
    )


def intake_section(story, styles):
    h1(story, "9. Intake, Color Sorting, Unjamming, and Scoring", styles)
    p(
        story,
        "Intake owns three intake motors and the optical sensor. The sorting step decides whether the visible object is alliance color or wrong color, counts accepted blocks, rejects wrong-color blocks, and triggers unjamming based on intake velocity drop.",
        styles,
    )
    snippet_flow(story, "Intake motor modes", "src/Intake.cpp", r"void Intake::intaking", 21, styles)
    snippet_flow(story, "Color sorting and block counting", "src/Intake.cpp", r"if \(var.correct_block_detected", 45, styles)
    snippet_flow(story, "Unjam trigger and priority", "src/Intake.cpp", r"float intakeVelocity", 58, styles)
    note_table(
        story,
        [
            FunctionNote("set_intake_power", "src/Intake.cpp", "Stores motor percentages.", "Like drivetrain, motor outputs are stored first and applied in main()."),
            FunctionNote("intaking / unjamming / color_sorting", "src/Intake.cpp", "Preset motor modes.", "Set motor power patterns for accepting, reversing/unjamming, or rejecting wrong color."),
            FunctionNote("score_high / score_mid", "src/Intake.cpp", "Scoring modes.", "Run motor combinations used by WaitAndScoreCommand."),
            FunctionNote("set_blocks_before_scoring", "src/Intake.cpp", "Sets scoring threshold.", "Clamps target block count to at least 1 unless the command only wants to reset count."),
            FunctionNote("intake_with_sorting_init", "src/Intake.cpp", "Resets sorting state.", "Loads config, sets default motor powers, resets object/color/timer/jam variables."),
            FunctionNote("intake_with_sorting_step", "src/Intake.cpp", "Main intake logic.", "Reads hue, classifies red/blue, counts accepted blocks once per object, times wrong-color rejection, detects jams from velocity drop, and chooses final motor mode by priority."),
            FunctionNote("IntakeWithSorting::execute", "include/IntakeWithSortingCommand.h", "Command wrapper.", "Calls intake.intake_with_sorting_step() each scheduler loop."),
            FunctionNote("WaitAndScoreCommand::execute", "include/WaitAndScoreCommand.h", "Timed scoring command.", "Waits for configured delay, runs score_high or score_mid for score_time, then stops the intake."),
            FunctionNote("SetBlocksBeforeScoringCommand::initialize", "include/SetBlocksBeforeScoringCommand.h", "Sets/reset count.", "Optionally sets blocks_before_scoring and resets accepted count when reset_count is true."),
        ],
        styles,
    )


def routines_section(story, styles):
    h1(story, "10. Autonomous Routine Assembly", styles)
    p(
        story,
        "Routines.cpp is the hub where configs, targets, command objects, conditional checks, and command sequences are assembled. AI_ROUTE_ONE repeats AI_ROUTE_1 forever, so after one collect/score cycle ends it starts again.",
        styles,
    )
    snippet_flow(story, "Main repeated autonomous route", "src/Routines.cpp", r"AI_ROUTE_1.addCommand\(&set_blocks_before_scoring", 22, styles)
    snippet_flow(story, "Scoring sequence retry before wall alignment", "src/Routines.cpp", r"score_bottom_left_sequence.addCommand", 50, styles)
    note_table(
        story,
        [
            FunctionNote("trackBlockWasSuccessful", "src/Routines.cpp", "Branch helper.", "Returns true only if the last TrackBlockCommand ended with TRACK_BLOCK_SUCCESS."),
            FunctionNote("hasEnoughBlocksToScore", "src/Routines.cpp", "Branch helper.", "Returns intake.has_enough_blocks_to_score()."),
            FunctionNote("robotIsOnTopSide / robotIsOnRightSide", "src/Routines.cpp", "Corner selector helpers.", "Use the current filtered/cached pose to choose which scoring corner sequence to run."),
            FunctionNote("robotIsCloseToPoint", "src/Routines.cpp", "Score approach retry check.", "Requires the latest filtered GPS command to have succeeded, then checks Euclidean distance to the approach point."),
            FunctionNote("score*ApproachIsClose", "src/Routines.cpp", "Per-corner retry conditions.", "Call robotIsCloseToPoint() with the relevant reflected score approach target."),
            FunctionNote("build_AI_routine", "src/Routines.cpp", "Builds command graph.", "Adds intake helpers to parallel groups, builds all four scoring sequences, then builds the repeated AI route."),
            FunctionNote("choose_score_corner", "src/Routines.cpp", "Nested conditional.", "First chooses top vs bottom by y, then left vs right by x."),
        ],
        styles,
    )


def function_reference(story, styles):
    h1(story, "11. Full Function Reference by File", styles)
    p(
        story,
        "This section is the compact function-by-function index. Earlier sections show the most important snippets; this index names what each function or item is responsible for.",
        styles,
    )
    groups: list[tuple[str, list[FunctionNote]]] = [
        (
            "Core command framework",
            [
                FunctionNote("Command::initialize", "include/Command.h", "Optional setup hook.", "Default empty implementation used before first execute()."),
                FunctionNote("Command::execute", "include/Command.h", "Required loop hook.", "Pure virtual; each command defines one scheduler-cycle slice."),
                FunctionNote("Command::isFinished", "include/Command.h", "Required completion hook.", "Pure virtual; scheduler removes the command once true."),
                FunctionNote("Command::end", "include/Command.h", "Optional cleanup hook.", "Default empty; most motor commands stop outputs here."),
                FunctionNote("Scheduler::removeCommand", "include/Scheduler.h", "Internal erase helper.", "Removes command and matching initialized flag."),
                FunctionNote("Scheduler::schedule", "include/Scheduler.h", "Queue command.", "Adds non-null command pointer with initialized=false."),
                FunctionNote("Scheduler::run", "include/Scheduler.h", "Command loop.", "Initializes, executes, ends, and removes finished commands."),
                FunctionNote("Scheduler::cancelAll", "include/Scheduler.h", "Stop all scheduled work.", "Ends initialized commands then clears vectors."),
                FunctionNote("Scheduler::isEmpty", "include/Scheduler.h", "Status helper.", "Returns whether any commands are scheduled."),
            ],
        ),
        (
            "Command groups",
            [
                FunctionNote("SequentialCommandGroup::addCommand", "src/SequentialCommandGroup.cpp", "Append child.", "Adds non-null command to list."),
                FunctionNote("SequentialCommandGroup::initialize", "src/SequentialCommandGroup.cpp", "Start sequence.", "Resets index and finished state."),
                FunctionNote("SequentialCommandGroup::execute", "src/SequentialCommandGroup.cpp", "Run current child.", "Initializes current child, executes it, ends it when finished, then advances."),
                FunctionNote("SequentialCommandGroup::end", "src/SequentialCommandGroup.cpp", "End active child.", "If interrupted, calls end() on current child."),
                FunctionNote("ParallelCommandGroup::execute", "src/ParallelCommandGroup.cpp", "Run all children.", "Executes unfinished children until all are complete."),
                FunctionNote("ParallelDeadlineGroup::execute", "src/ParallelDeadlineGroup.cpp", "Run deadline and helpers.", "Group finishes when deadline finishes."),
                FunctionNote("RaceCommandGroup::execute", "src/RaceCommandGroup.cpp", "Race children.", "Group finishes as soon as any child finishes."),
                FunctionNote("ConditionalCommandGroup::initialize", "src/ConditionalCommandGroup.cpp", "Select branch.", "Chooses trueCommand or falseCommand and initializes it."),
                FunctionNote("RepeatForeverCommandGroup::execute", "src/RepeatForeverCommandGroup.cpp", "Restart wrapped command.", "When child finishes, ends and reinitializes it."),
            ],
        ),
        (
            "Drivetrain and pose",
            [
                FunctionNote("main", "src/main.cpp", "Program entry point.", "Sets up Jetson, builds/schedules autonomous, waits for calibration, seeds odom from GPS, then loops scheduler/motors/display."),
                FunctionNote("Drivetrain::get_pose", "src/Drivetrain.cpp", "Return cached pose.", "Returns the internal Pose struct."),
                FunctionNote("Drivetrain::set_odom_pose", "src/Drivetrain.cpp", "Set odom pose.", "Stores x/y/theta and updates inertial heading."),
                FunctionNote("Drivetrain::set_heading_degrees", "src/Drivetrain.cpp", "Set inertial heading safely.", "Rejects invalid values, normalizes, then writes to sensor."),
                FunctionNote("Drivetrain::update_odom_pose", "src/Drivetrain.cpp", "Refresh heading.", "Copies inertial heading into pose.theta."),
                FunctionNote("Drivetrain::set_drive_power", "src/Drivetrain.cpp", "Store drive command.", "Updates left_drive_power/right_drive_power."),
                FunctionNote("Drivetrain::apply_motor_power", "src/Drivetrain.cpp", "Spin motors.", "Applies stored power to motor groups."),
                FunctionNote("Drivetrain::stop", "src/Drivetrain.cpp", "Brake drivetrain.", "Zeros power and calls stop(brake)."),
                FunctionNote("PositionTracking::normalize_heading", "src/PositionTracking.cpp", "Normalize heading.", "Wraps heading to 0..360."),
                FunctionNote("PositionTracking::get_raw_x/y", "src/PositionTracking.cpp", "Corrected GPS x/y.", "Converts sensor position to robot-center position using heading and sensor offset."),
                FunctionNote("PositionTracking::get_raw_heading", "src/PositionTracking.cpp", "Corrected heading convention.", "Returns GPS heading plus 180 normalized."),
                FunctionNote("PositionTracking::update_raw_pose", "src/PositionTracking.cpp", "Cache GPS pose.", "Stores corrected raw x/y/heading."),
                FunctionNote("PositionTracking::set_pose", "src/PositionTracking.cpp", "Set filtered pose.", "Stores filtered x/y/heading into gpsPose."),
                FunctionNote("PositionTracking getters", "src/PositionTracking.cpp", "Access pose fields.", "Return cached x, y, or heading."),
            ],
        ),
        (
            "Vision and block tracking",
            [
                FunctionNote("JetsonSerial::JetsonSerialSetup", "src/JetsonSerial.cpp", "Open serial input.", "Opens /dev/serial1, reports status on Brain screen, seeds rand."),
                FunctionNote("JetsonSerial::clear_blocks", "src/JetsonSerial.cpp", "Reset block arrays.", "Clears block count, selected block, and arrays."),
                FunctionNote("JetsonSerial::parse_xy_pair", "src/JetsonSerial.cpp", "Parse x,y token.", "Splits a string pair and converts to integers."),
                FunctionNote("JetsonSerial::update_block_pose", "src/JetsonSerial.cpp", "Read Jetson frame.", "Parses 0, x,y, or count;x,y;... messages."),
                FunctionNote("FindBlockCommand helper functions", "include/FindBlockCommand.h", "Allowed find/center logic.", "Estimate field position, filter avoid zones, select allowed candidates, monitor search/centering progress, and run state machine."),
                FunctionNote("TrackBlockCommand helper functions", "include/TrackBlockCommand.h", "Allowed tracking logic.", "Track only acceptable blocks, drive with PID, and use circular arc unstuck sequence when progress stalls."),
            ],
        ),
        (
            "Movement and scoring commands",
            [
                FunctionNote("DriveStraightCommand::execute", "include/DriveStraightCommand.h", "Distance and heading PID.", "Uses encoder distance and inertial heading to drive a straight target."),
                FunctionNote("InertialTurnCommand::execute", "include/InertialTurnCommand.h", "Turn PID.", "Turns to target heading with settle time and unstuck support."),
                FunctionNote("DriveToX/YPositionCommand::execute", "include/DriveToXPositionCommand.h / DriveToYPositionCommand.h", "Axis target drive.", "Uses GPS axis error and heading correction."),
                FunctionNote("DriveToPointCommand helpers", "include/DriveToPointCommand.h", "Point-drive state machine.", "Compute target heading/distance, point first, drive second, and optionally escape with arcs."),
                FunctionNote("DriveToPointUntilX/YCommand helpers", "include/DriveToPointUntilXCommand.h / include/DriveToPointUntilYCommand.h", "Exit-line state machine.", "Same as drive-to-point, but finish when crossing an x or y line."),
                FunctionNote("GrabBlockCommand::execute", "include/GrabBlockCommand.h", "Final block grab motion.", "Short controlled drive after tracking aligns to a block."),
                FunctionNote("WallAlignmentCommand::execute", "include/WallAlignmentCommand.h", "Wall distance alignment.", "Uses rear distance sensors and GPS heading to back to target wall distance while correcting heading."),
                FunctionNote("UnstuckArcHelper functions", "include/UnstuckArcHelper.h", "Reusable escape support.", "Configure, monitor progress, start, run, and fail a four-stage circular arc escape."),
            ],
        ),
        (
            "Intake and scoring",
            [
                FunctionNote("Intake::intake_with_sorting_step", "src/Intake.cpp", "Color sorting state machine.", "Reads optical sensor, classifies color, counts accepted blocks, rejects wrong color, and unjams on velocity drop."),
                FunctionNote("Intake::apply_motor_power", "src/Intake.cpp", "Spin intake motors.", "Applies stored powers to initial/middle/final intake motors."),
                FunctionNote("IntakeWithSorting::execute", "include/IntakeWithSortingCommand.h", "Sorting command wrapper.", "Runs one intake sorting step."),
                FunctionNote("WaitAndScoreCommand::execute", "include/WaitAndScoreCommand.h", "Timed scoring.", "Waits, scores, then stops."),
                FunctionNote("SetBlocksBeforeScoringCommand::initialize", "include/SetBlocksBeforeScoringCommand.h", "Configure score threshold.", "Sets target block count and optionally resets accepted count."),
            ],
        ),
        (
            "Routines and debug",
            [
                FunctionNote("trackBlockWasSuccessful", "src/Routines.cpp", "Track result condition.", "Used to decide whether to run GrabBlockCommand."),
                FunctionNote("hasEnoughBlocksToScore", "src/Routines.cpp", "Score condition.", "Used to decide whether to run scoring sequence."),
                FunctionNote("robotIsOnTopSide / robotIsOnRightSide", "src/Routines.cpp", "Field side conditions.", "Choose the correct reflected scoring corner."),
                FunctionNote("robotIsCloseToPoint and score*ApproachIsClose", "src/Routines.cpp", "Pre-wall-alignment retry checks.", "Use latest filtered GPS pose to decide whether to retry approach drive."),
                FunctionNote("build_AI_routine", "src/Routines.cpp", "Command graph builder.", "Creates parallel intake groups, scoring sequences, and repeated autonomous sequence."),
                FunctionNote("setCommandStatus / getCommandStatus", "src/CommandStatus.cpp", "Debug text storage.", "Stores the command label printed on Brain row 12."),
                FunctionNote("setLastWallAlignmentDistance / getLastWallAlignmentDistance", "src/CommandStatus.cpp", "Wall distance display.", "Stores the last distance sensor reading printed on Brain row 7."),
            ],
        ),
    ]
    for title, rows in groups:
        h2(story, title, styles)
        note_table(story, rows, styles)


def explanation_notes(story, styles):
    h1(story, "12. Explanation Checklist for Presenting the Code", styles)
    p(story, "Use this checklist when explaining the code to someone else.", styles)
    bullet(story, "Start with the scheduler: the robot is not a single blocking while-loop; each command advances one slice per scheduler cycle.", styles)
    bullet(story, "Explain the autonomous loop: set block target, GPS update, find block, GPS update, track/intake, grab if track succeeded, score if enough blocks.", styles)
    bullet(story, "Explain pose: GPS readings are corrected from the physical sensor location to robot-center coordinates before other commands use them.", styles)
    bullet(story, "Explain block filtering: visible blocks are converted to estimated field positions; avoid-zone blocks are skipped before selecting a target.", styles)
    bullet(story, "Explain find behavior: normal find rotates in place; circular arc unstuck only starts if heading does not change while trying to rotate.", styles)
    bullet(story, "Explain tracking behavior: x error controls turn, y error controls forward motion, and progress is measured by encoder, heading, and y-error changes.", styles)
    bullet(story, "Explain scoring: after driving toward the scoring approach point, the robot re-runs filtered GPS and retries the drive once if it is not close enough.", styles)
    bullet(story, "Explain wall alignment: rear sensors report wall distance, heading is corrected continuously, and the robot backs toward the wall if neither sensor sees it.", styles)
    bullet(story, "Explain intake: color sorting rejects wrong color, counts accepted blocks, and can unjam based on velocity drop even if it is not currently detecting a block.", styles)
    bullet(story, "Explain debug screen: selected block, errors, wall distance, GPS/IMU headings, and current command status are printed every loop.", styles)


def build():
    styles = build_styles()
    doc = CodeReferenceDoc(str(OUT))
    story = []

    story.append(Paragraph("VexAI1 Code Reference Manual", styles["TitleBlue"]))
    story.append(
        Paragraph(
            escape(
                f"Generated {date.today().isoformat()} from the current workspace at {ROOT}. "
                "This manual explains the autonomous command architecture, major state machines, and function-level responsibilities."
            ),
            styles["RefSubtitle"],
        )
    )
    p(
        story,
        "Design preset: compact_reference_guide. The document uses prose sections for concepts, compact function tables for reference, and code snippets for the control paths you are most likely to need to explain.",
        styles,
    )
    story.append(Spacer(1, 10))
    h2(story, "Table of Contents", styles)
    for item in [
        "1. System Overview",
        "2. Command Framework",
        "3. Hardware, Pose, and Debug Status",
        "4. Jetson Vision Data Path",
        "5. Field Avoid Zones and FindBlockCommand",
        "6. TrackBlockCommand",
        "7. Movement Commands",
        "8. GPS Filtering and Wall Alignment",
        "9. Intake, Color Sorting, Unjamming, and Scoring",
        "10. Autonomous Routine Assembly",
        "11. Full Function Reference by File",
        "12. Explanation Checklist",
    ]:
        bullet(story, item, styles)
    story.append(PageBreak())

    overview(story, styles)
    command_framework(story, styles)
    hardware_pose(story, styles)
    jetson_vision(story, styles)
    field_avoid_and_find(story, styles)
    tracking_section(story, styles)
    movement_section(story, styles)
    wall_and_gps_section(story, styles)
    intake_section(story, styles)
    routines_section(story, styles)
    function_reference(story, styles)
    explanation_notes(story, styles)

    doc.build(story)


if __name__ == "__main__":
    os.makedirs(OUT.parent, exist_ok=True)
    build()
    print(OUT)
