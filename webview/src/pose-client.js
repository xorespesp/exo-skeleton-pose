// WebSocket client for the exo-skeleton-pose protocol. (pose_protocol.fbs)

import * as flatbuffers from 'flatbuffers';
import {
    Message, Payload,
    OpenSourceStream, CloseSourceStream, CalibrateRestPose, ClearRestPose, GetServerStatus,
    PoseFrame, ServerStatus, SourceStreamEnded, Ack,
} from './generated/exo/proto.js';

// request_id value reserved for messages the server sends on its own
// (status/pose/ended notifications); clients must use a non-zero id.
const RESERVED_SERVER_NOTIFY_REQ_ID = 0;

export class PoseClient {
    constructor() {
        this.ws = null;
        this._nextRequestId = RESERVED_SERVER_NOTIFY_REQ_ID + 1; // stamped on each command, echoed back in the Ack
        // Callbacks (assign after constructing): (obj) => void
        this.onOpen = null;
        this.onClose = null;
        this.onPoseFrame = null;
        this.onStatus = null;
        this.onSourceEnded = null;
        this.onAck = null; // (ack, requestId) => void
    }

    connect(url) {
        this.ws = new WebSocket(url);
        this.ws.binaryType = 'arraybuffer';
        this.ws.onopen = () => this.onOpen?.();
        this.ws.onclose = () => this.onClose?.();
        this.ws.onerror = () => this.ws?.close();
        this.ws.onmessage = (ev) => this._decode(new Uint8Array(ev.data));
    }

    disconnect() {
        this.ws?.close();
    }

    // --- commands (client -> server) ---------------------------------------------

    // opts: { source, tagSizeM?, exposureUs?, gain? }. Null exposure/gain == auto.
    sendOpen({ source, tagSizeM = 0.05, exposureUs = null, gain = null }) {
        const b = new flatbuffers.Builder(256);
        const srcOff = b.createString(source);
        OpenSourceStream.startOpenSourceStream(b);
        OpenSourceStream.addSource(b, srcOff);
        OpenSourceStream.addTagSizeM(b, tagSizeM);
        if (exposureUs !== null) { OpenSourceStream.addExposureUs(b, exposureUs); }
        if (gain !== null) { OpenSourceStream.addGain(b, gain); }
        this._send(b, Payload.OpenSourceStream, OpenSourceStream.endOpenSourceStream(b));
    }

    sendClose() {
        const b = new flatbuffers.Builder(64);
        CloseSourceStream.startCloseSourceStream(b);
        this._send(b, Payload.CloseSourceStream, CloseSourceStream.endCloseSourceStream(b));
    }

    sendCalibrateRestPose() {
        const b = new flatbuffers.Builder(64);
        CalibrateRestPose.startCalibrateRestPose(b);
        this._send(b, Payload.CalibrateRestPose, CalibrateRestPose.endCalibrateRestPose(b));
    }

    sendClearRestPose() {
        const b = new flatbuffers.Builder(64);
        ClearRestPose.startClearRestPose(b);
        this._send(b, Payload.ClearRestPose, ClearRestPose.endClearRestPose(b));
    }

    // Request the current status; the server replies with a ServerStatus. (no Ack)
    sendGetServerStatus() {
        const b = new flatbuffers.Builder(64);
        GetServerStatus.startGetServerStatus(b);
        this._send(b, Payload.GetServerStatus, GetServerStatus.endGetServerStatus(b));
    }

    // --- internals ----------------------------------------------------------------

    _send(b, payloadType, payloadOffset) {
        const requestId = this._nextRequestId++;
        Message.startMessage(b);
        Message.addPayloadType(b, payloadType);
        Message.addPayload(b, payloadOffset);
        Message.addRequestId(b, requestId);
        b.finish(Message.endMessage(b));
        if (this.ws?.readyState === WebSocket.OPEN) { this.ws.send(b.asUint8Array()); }
        return requestId;
    }

    _decode(bytes) {
        const msg = Message.getRootAsMessage(new flatbuffers.ByteBuffer(bytes));
        const requestId = msg.requestId();
        switch (msg.payloadType()) {
            case Payload.PoseFrame:    this.onPoseFrame?.(msg.payload(new PoseFrame())); break;
            case Payload.ServerStatus: this.onStatus?.(msg.payload(new ServerStatus())); break;
            case Payload.SourceStreamEnded: this.onSourceEnded?.(msg.payload(new SourceStreamEnded())); break;
            case Payload.Ack:          this.onAck?.(msg.payload(new Ack()), requestId); break;
            default: break;
        }
    }
}
