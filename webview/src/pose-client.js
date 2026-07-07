// WebSocket client for the exo-skeleton-pose protocol. (pose_protocol.fbs)

import * as flatbuffers from 'flatbuffers';
import {
  Message, Payload,
  OpenSource, CloseSource, CalibrateRestPose, ClearRestPose, GetServerStatus,
  PoseFrame, ServerStatus, Ack,
} from './protocol/exo/proto.js';

export class PoseClient {
  constructor() {
    this.ws = null;
    // Callbacks (assign after constructing): (obj) => void
    this.onOpen = null;
    this.onClose = null;
    this.onPoseFrame = null;
    this.onStatus = null;
    this.onAck = null;
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
    OpenSource.startOpenSource(b);
    OpenSource.addSource(b, srcOff);
    OpenSource.addTagSizeM(b, tagSizeM);
    if (exposureUs !== null) { OpenSource.addExposureUs(b, exposureUs); }
    if (gain !== null) { OpenSource.addGain(b, gain); }
    this._send(b, Payload.OpenSource, OpenSource.endOpenSource(b));
  }

  sendClose() {
    const b = new flatbuffers.Builder(64);
    CloseSource.startCloseSource(b);
    this._send(b, Payload.CloseSource, CloseSource.endCloseSource(b));
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
    Message.startMessage(b);
    Message.addPayloadType(b, payloadType);
    Message.addPayload(b, payloadOffset);
    b.finish(Message.endMessage(b));
    if (this.ws?.readyState === WebSocket.OPEN) { this.ws.send(b.asUint8Array()); }
  }

  _decode(bytes) {
    const msg = Message.getRootAsMessage(new flatbuffers.ByteBuffer(bytes));
    switch (msg.payloadType()) {
      case Payload.PoseFrame:    this.onPoseFrame?.(msg.payload(new PoseFrame())); break;
      case Payload.ServerStatus: this.onStatus?.(msg.payload(new ServerStatus())); break;
      case Payload.Ack:          this.onAck?.(msg.payload(new Ack())); break;
      default: break;
    }
  }
}
