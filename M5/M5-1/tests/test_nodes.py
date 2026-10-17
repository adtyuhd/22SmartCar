
import threading
import time

from task_fsm.context import TaskContext
from task_fsm.core import TaskScheduler
from task_fsm.nodes import (
    WaitForTrigger,
    NavigateTo,
    DetectTarget,
    ExecuteGrasp,
    SpeakTTS,
)


def make_context():
    return TaskContext("node_test")


# ==================== WaitForTrigger ====================

def test_wait_for_trigger_success():
    ctx = make_context()
    node = WaitForTrigger("wait", {"timeout": 0.1}, ctx)

    node.trigger()

    assert node.execute() == "SUCCESS"


def test_wait_for_trigger_timeout():
    ctx = make_context()
    node = WaitForTrigger("wait", {"timeout": 0.01}, ctx)

    assert node.execute() == "FAILED"


def test_wait_for_trigger_cancel():
    ctx = make_context()
    node = WaitForTrigger("wait", {"timeout": 5}, ctx)
    results = []

    thread = threading.Thread(
        target=lambda: results.append(node.execute())
    )
    thread.start()

    time.sleep(0.02)
    node.cancel()
    thread.join(timeout=0.5)

    assert not thread.is_alive()
    assert results == ["SKIPPED"]


# ==================== NavigateTo ====================

def test_navigate_to_success():
    ctx = make_context()
    pose = {"x": 1.0, "y": 2.0, "yaw": 0.0}

    node = NavigateTo(
        "navigate",
        {
            "pose": pose,
            "client": lambda p: "arrived",
        },
        ctx,
    )

    assert node.execute() == "SUCCESS"
    assert ctx.data["last_nav_result"] == "arrived"


def test_navigate_to_failure():
    ctx = make_context()

    def failed_client(pose):
        raise RuntimeError("navigation failed")

    node = NavigateTo(
        "navigate",
        {
            "pose": {"x": 1, "y": 2, "yaw": 0},
            "client": failed_client,
        },
        ctx,
    )

    assert node.execute() == "FAILED"


def test_navigate_to_timeout():
    ctx = make_context()

    def timeout_client(pose):
        raise TimeoutError("navigation timed out")

    node = NavigateTo(
        "navigate",
        {
            "pose": {"x": 1, "y": 2, "yaw": 0},
            "client": timeout_client,
        },
        ctx,
    )

    assert node.execute() == "FAILED"


# ==================== DetectTarget ====================

def test_detect_target_success():
    ctx = make_context()
    node = DetectTarget(
        "detect",
        {
            "target_class": "qrcode",
            "min_confidence": 0.7,
            "timeout": 0.01,
        },
        ctx,
    )

    node.on_detection({
        "class": "qrcode",
        "confidence": 0.9,
        "id": "qr_01",
        "pose": [2.1, 0.4],
    })

    assert node.execute() == "SUCCESS"
    assert ctx.data["target_id"] == "qr_01"
    assert ctx.data["target_pose"] == [2.1, 0.4]


def test_detect_target_confidence_too_low():
    ctx = make_context()
    node = DetectTarget(
        "detect",
        {
            "target_class": "qrcode",
            "min_confidence": 0.7,
            "timeout": 0.01,
        },
        ctx,
    )

    node.on_detection({
        "class": "qrcode",
        "confidence": 0.5,
        "id": "qr_02",
        "pose": [1.0, 1.0],
    })

    assert node.execute() == "FAILED"


def test_detect_target_timeout():
    ctx = make_context()
    node = DetectTarget(
        "detect",
        {
            "target_class": "qrcode",
            "min_confidence": 0.7,
            "timeout": 0.01,
        },
        ctx,
    )

    assert node.execute() == "FAILED"


def test_detect_target_cancel():
    ctx = make_context()
    node = DetectTarget(
        "detect",
        {
            "target_class": "qrcode",
            "timeout": 5,
        },
        ctx,
    )
    results = []

    thread = threading.Thread(
        target=lambda: results.append(node.execute())
    )
    thread.start()

    time.sleep(0.02)
    node.cancel()
    thread.join(timeout=0.5)

    assert not thread.is_alive()
    assert results == ["SKIPPED"]


# ==================== ExecuteGrasp ====================

def test_execute_grasp_success():
    ctx = make_context()
    ctx.data["grasp_pose"] = [1.0, 2.0, 3.0]

    node = ExecuteGrasp(
        "grasp",
        {
            "grasp_type": "top_down",
            "client": lambda pose, grasp_type: {
                "success": True,
                "object_pose": [1.1, 2.1, 3.1],
            },
        },
        ctx,
    )

    assert node.execute() == "SUCCESS"
    assert ctx.data["grasp_result"]["success"] is True
    assert ctx.data["object_pose"] == [1.1, 2.1, 3.1]


def test_execute_grasp_failure():
    ctx = make_context()
    ctx.data["grasp_pose"] = [1.0, 2.0, 3.0]

    node = ExecuteGrasp(
        "grasp",
        {
            "grasp_type": "top_down",
            "client": lambda pose, grasp_type: {
                "success": False,
                "object_pose": pose,
            },
        },
        ctx,
    )

    assert node.execute() == "FAILED"


def test_execute_grasp_missing_input():
    ctx = make_context()

    node = ExecuteGrasp(
        "grasp",
        {
            "grasp_type": "top_down",
            "client": lambda pose, grasp_type: {
                "success": True,
                "object_pose": [1, 2, 3],
            },
        },
        ctx,
    )

    assert node.execute() == "FAILED"


def test_execute_grasp_retry_success():
    ctx = make_context()
    ctx.data["grasp_pose"] = [1.0, 2.0, 3.0]
    calls = []

    def grasp_client(pose, grasp_type):
        calls.append(1)

        if len(calls) == 1:
            return {
                "success": False,
                "object_pose": pose,
            }

        return {
            "success": True,
            "object_pose": [1.1, 2.1, 3.1],
        }

    node = ExecuteGrasp(
        "grasp",
        {
            "grasp_type": "top_down",
            "retry": 1,
            "client": grasp_client,
        },
        ctx,
    )

    scheduler = TaskScheduler([node], ctx)

    assert scheduler.run() == "SUCCESS"
    assert len(calls) == 2
    assert scheduler.step_records[0]["retries"] == 1


def test_execute_grasp_retry_exhausted():
    ctx = make_context()
    ctx.data["grasp_pose"] = [1.0, 2.0, 3.0]
    calls = []

    def grasp_client(pose, grasp_type):
        calls.append(1)
        return {
            "success": False,
            "object_pose": pose,
        }

    node = ExecuteGrasp(
        "grasp",
        {
            "grasp_type": "top_down",
            "retry": 2,
            "client": grasp_client,
        },
        ctx,
    )

    scheduler = TaskScheduler([node], ctx)

    assert scheduler.run() == "FAILED"
    assert len(calls) == 3
    assert scheduler.step_records[0]["retries"] == 2


# ==================== SpeakTTS ====================

def test_speak_tts_success():
    ctx = make_context()
    spoken = []

    node = SpeakTTS(
        "speak",
        {
            "text": "任务开始",
            "client": lambda text: spoken.append(text),
        },
        ctx,
    )

    assert node.execute() == "SUCCESS"
    assert spoken == ["任务开始"]


def test_speak_tts_missing_client():
    ctx = make_context()
    node = SpeakTTS(
        "speak",
        {"text": "任务开始"},
        ctx,
    )

    assert node.execute() == "FAILED"


def test_speak_tts_timeout():
    ctx = make_context()

    def timeout_client(text):
        raise TimeoutError("TTS timed out")

    node = SpeakTTS(
        "speak",
        {
            "text": "任务开始",
            "client": timeout_client,
        },
        ctx,
    )

    assert node.execute() == "FAILED"
