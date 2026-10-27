# PeMS access without blocking the twin build

27 October 2026. Sam has no account or District 7 downloads. No application has
been submitted by Codex; Sam enters his details and reviews the terms personally.

## What PeMS is for

Caltrans Performance Measurement System supplies traffic observations and
historical archives. We want speeds, flows, occupancy and detector-quality
information for I-110 and its ramps to answer: does our simulated traffic break
down, queue and recover like this actual road under comparable demand?

PeMS is **not required** to build geometry, open the viewer, test road contact,
exercise synthetic incidents or develop a driving controller. It does not supply
our 3D road mesh or replace camera/lidar perception. Aggregate detector readings
also do not establish individual human gap acceptance or every upstream queue.
Those require additional observations. Real traffic evidence is needed for a
claim of local traffic fidelity; PeMS is one source, not the only possible one.

## Application walkthrough

1. Open [Apply for an Account](https://pems.dot.ca.gov/?dnode=apply).
2. Enter your first/last name, USC email and matching email confirmation.
3. Institution: **University of Southern California**. Enter further affiliation
   details only if accurate; do not invent an advisor or department.
4. Suggested reason, editable by Sam:

   > Academic research at USC developing and evaluating a digital twin of
   > southbound I-110 near MLK Boulevard and Vernon Avenue. I plan to use historical
   > District 7 mainline and ramp traffic measurements and detector-quality data
   > to calibrate and validate offline traffic simulations.

5. Review the displayed terms/privacy information, choose the appropriate consent
   response yourself and submit. The form does not require exiD access or payment.
6. Watch the submitted email account and spam folder for the approval email.
   The current application says credentials arrive by email; change the initial
   password after login. Keep credentials out of Git and shared configuration.

Caltrans describes registration as free and usually approved within **one to two
business days**. That is a stated typical interval, not a guarantee. If approval
does not arrive, contact **pems.support@dot.ca.gov**; no email was sent here.
Sources checked 27 October: [Caltrans access overview](https://dot.ca.gov/programs/traffic-operations/mpr/pems-source)
and the [live application](https://pems.dot.ca.gov/?dnode=apply). Some features are
restricted to Caltrans accounts, so an academic account is not a promise of every
available feed. We need historical research exports, not a commercial live feed.

## First small data request after access

Find District 7, route 110, southbound, around MLK and Vernon. Start with station
metadata and lane/ramp inventory, then a small date window with detector health
and observed/imputed fractions. Inspect station IDs and coordinate/postmile
conventions before requesting values. Do not assume every nearby station works.
Retain timestamps, time zone/DST treatment, units, processing version and original
files. No claim is made about an authenticated menu that has not been inspected.

After metadata qualification, reserve independent audit days before inspecting
their outcome values. Request development speed/flow/occupancy at mainline and
ramp stations for representative lower-demand, buildup and recovery conditions.
Match roadway epoch and incident context; do not treat annual ramp ADT as peak
inflow. Expand beyond the rendered tile to observe upstream and downstream queues.

## Architecture that avoids a service dependency

The current scene and event code have no PeMS import, credentials, network call
or account prerequisite. Future calibration reads **versioned local files**:

`source export → quality/provenance check → local calibrated scenario → offline runs`

The data adapter should accept the same documented schema from PeMS exports,
agency-provided counts or qualified local observations. Cache original files,
hashes and derived scenario versions. Fetching new data is an explicit research
step; neither replay nor training should wait on the PeMS website.

While access is pending or quality is poor, continue road/vehicle work with
clearly labeled synthetic demand. Public ramp counts can give broad plausibility
checks but several relevant counts are stale. Other agency datasets or a planned
local observation campaign are alternatives. If no suitable real observations
exist, retain a synthetic traffic experiment claim instead of asserting a
calibrated I-110 twin. This is an evidence limitation, not a reason to block
construction or introduce PeMS as a mandatory runtime dependency.
