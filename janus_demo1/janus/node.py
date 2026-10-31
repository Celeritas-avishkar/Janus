"""Glue between a brain (pure logic) and its protocol Endpoint."""

UNRELIABLE_MAX_AGE = 3.0


def pump(brain, ep, now, gcs=False):
    """Move brain.outbox into the endpoint."""
    out, brain.outbox = brain.outbox, []
    for item in out:
        kind, name, payload, key = item[0], item[1], item[2], item[3]
        rid = item[4] if len(item) > 4 else None
        if kind == "unrel":
            ep.send(name, payload, now)
        elif gcs and name == "MISSION":
            ep.send_reliable(name, payload, now, key=None, max_attempts=40,
                             on_done=lambda mid, rtt, att, r=rid: brain.on_upload_delivered(r, rtt, att),
                             on_fail=lambda mid, r=rid: brain.on_upload_failed(r))
        else:
            ep.send_reliable(name, payload, now, key=key, max_attempts=None)
    ep.tick(now)
    ep.purge_partial(now)


def deliver(brain, ep, data, now):
    for name, obj, _f in ep.on_datagram(data, now, max_age=UNRELIABLE_MAX_AGE):
        brain.on_message(name, obj, now)
