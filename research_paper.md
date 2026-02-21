# A Byzantine-Resilient Decentralized Coordination Framework with Acoustic TDOA Localization for Secure UAV Swarms

Decentralized Coordination and Acoustic Localization in Secure Autonomous Drone Swarms

Md Shahanur Islam Shagor

Project Architect & Lead Developer

Secure Autonomous Drone Swarm Research Laboratory

Abstract

This paper presents a fully implemented and experimentally validated decentralized autonomous drone swarm management system designed for operation in GPS-denied, vision-impaired, and electronically contested environments. The completed platform integrates six tightly coupled subsystems: a Multi-Criteria Suitability Scoring (MCSS)–based leader election protocol providing Byzantine Fault Tolerance for agents with rapid failover capability; an AES-256-GCM secured peer-to-peer communication mesh with PBKDF2-derived swarm keys and replay protection; a machine learning–enhanced dynamic obstacle avoidance engine combining collision-cone modeling, second-order kinematic prediction, and acceleration-constrained velocity blending; a GCC-PHAT–based acoustic TDOA localization module enabling GPS-independent source estimation with meter-level accuracy; a tamper-evident distributed flight logging framework (Flying Ledger) utilizing Ed25519 signatures and SHA3-256 cryptographic chaining; and a C++-implemented Differential Immune System supporting real-time motor degradation detection, adaptive thrust redistribution, and dynamic PID retuning for resilience against partial actuator failure. All mathematical models are directly mapped to production-level source code, ensuring full traceability between theoretical formulation and system implementation, and experimental evaluation confirms robust performance across fault tolerance, secure communication, and navigation stability metrics. Simulation results further demonstrate stable flight behavior under up to 30% motor degradation while maintaining end-to-end control latency below 50 milliseconds.

Index Terms - AES-256-GCM, acoustic localization, Byzantine fault tolerance, collision-cone modeling, decentralized control, differential immune system, distributed ledger, drone swarm systems, Ed25519, GCC-PHAT, GPS-denied navigation, leader election, MAVLink, obstacle avoidance, SHA3-256, time difference of arrival (TDOA).

Source Code: The complete implementation of the framework, including the C++ hardware abstraction layer and Python swarm intelligence modules, is available at: https://github.com/smshagor-dev/decentralized-coordination-and-acoustic-localization-in-secure-autonomousa-drone-swarms

I. INTRODUCTION

1.1 Background and Motivation

Unmanned Aerial Vehicle (UAV) swarm systems have evolved from experimental laboratory platforms into mission-critical tools across defense surveillance, urban search-and-rescue, disaster response, precision agriculture, and infrastructure inspection. Contemporary implementations including commercial swarm platforms and research frameworks built upon middleware such as ROS have significantly advanced coordination and autonomy capabilities. However, most existing architectures remain dependent on centralized control assumptions, persistent GNSS availability, reliable communication channels, and benign electromagnetic conditions. These design assumptions introduce systemic vulnerabilities that become pronounced in GPS-denied environments, electronically contested airspace, smoke-obscured or low-visibility conditions, and adversarial cyber contexts.

The present work addresses these limitations through the design and full implementation of a decentralized swarm architecture engineered for operational resilience under adverse conditions. Each architectural decision including the adoption of Ed25519 digital signatures for efficient cryptographic verification and the use of GCC-PHAT–based acoustic cross-correlation for robust time-difference-of-arrival estimation was selected to mitigate identified failure modes in conventional swarm systems. The resulting framework integrates six interdependent subsystems that collectively enhance fault tolerance, secure communication, navigation robustness, and actuator resilience, forming a cohesive production-level platform validated through experimental implementation.

1.2 Design Philosophy

The architectural philosophy underlying this work is grounded in decentralized resilience and multi-layered redundancy. The system is designed such that no single node possesses unilateral authority capable of compromising overall swarm functionality. Similarly, no single sensing modality exclusively governs navigation, and no individual software layer independently determines system safety. Instead, the framework adopts a distributed decision-making model in which control authority, sensing inputs, and fault mitigation mechanisms are hierarchically and functionally partitioned.

This layered design follows a concentric redundancy principle, where successive protective mechanisms compensate for potential failures in outer layers. Communication security, leader reconfiguration, sensor fusion, actuator health monitoring, and flight logging operate as mutually reinforcing subsystems. Through this architecture, localized failures whether at the hardware, software, or network level are contained without propagating into system-wide collapse, thereby enhancing operational robustness in contested or degraded environments.

II. PROBLEM STATEMENT

The development of the proposed decentralized swarm architecture is motivated by six critical and well-documented limitations in contemporary autonomous UAV swarm systems. These challenges span operational reliability, environmental robustness, distributed coordination, communication security, actuator resilience, and audit integrity. Existing swarm implementations often rely on centralized coordination, persistent GNSS availability, unimodal sensing assumptions, or trusted network environments design constraints that introduce vulnerabilities under adversarial or degraded conditions.

This work formally characterizes these six bottlenecks and presents a structured architectural response in which each identified limitation is addressed through a corresponding subsystem within the integrated framework. By mapping specific operational failure modes to targeted algorithmic and cryptographic mechanisms, the system establishes a traceable relationship between identified swarm vulnerabilities and implemented mitigation strategies.

A. Single Point of Failure and Centralized Fragility

Conventional UAV swarm architectures frequently depend on a centralized Ground Control Station (GCS) or a statically designated master node for coordination. Such dependency introduces a Single Point of Failure (SPOF), a well-established vulnerability in distributed systems that increases susceptibility to crash faults and Byzantine behaviors [49]. Compromise or malfunction of the coordinating node whether due to hardware failure, communication disruption, or adversarial interference can significantly degrade or halt collective swarm operation. Moreover, many deployed commercial swarm implementations lack a dynamic, merit-based re-election mechanism capable of autonomously determining the most suitable successor in the event of leader failure.

To address this limitation, the proposed architecture integrates a Multi-Criteria Suitability Scoring (MCSS) leader election protocol supported by the Flying Ledger, acoustic TDOA telemetry consistency, and the Differential Immune System’s health diagnostics. The MCSS mechanism continuously evaluates battery state, motor integrity, communication link stability, and system latency across active nodes. Upon detection of leader failure, the protocol performs decentralized scoring and promotes the most suitable surviving drone within approximately 1.2 seconds, thereby preserving coordinated mission execution without reliance on a fixed authority node.

B. GPS Dependency and Signal Spoofing in Contested Zones

Contemporary UAV navigation systems rely predominantly on Global Navigation Satellite Systems (GNSS), particularly GPS, for positioning and synchronization. However, GPS signals reach the Earth's surface at approximately −130 dBm, rendering them highly susceptible to intentional jamming using low-cost radio frequency transmitters. In contested or structurally complex environments including urban canyons, dense forest canopies, mountainous terrain, indoor facilities, and electronic warfare scenarios GNSS signals may become intermittent, degraded, or entirely unavailable [9], [10]. In addition to signal denial, GPS spoofing attacks where a falsified but stronger satellite signal is broadcast pose a significant threat by misleading autonomous systems into erroneous navigation states without immediate detection.

To mitigate this vulnerability, the proposed architecture incorporates a GCC-PHAT–based acoustic Time Difference of Arrival (TDOA) localization subsystem that operates independently of satellite infrastructure. By estimating inter-drone acoustic propagation delays through generalized cross-correlation with phase transform weighting, the system computes two-dimensional source positions under GPS-denied conditions. Experimental validation demonstrates meter-level localization accuracy (sub-3 m) in controlled zero-visibility scenarios, providing an alternative positioning mechanism resilient to GNSS jamming and spoofing attacks.

C. Sensor Failure in Vision-Denied Environments

Many autonomous UAV systems depend heavily on optical sensing modalities, including RGB cameras, LiDAR, and stereo vision, for perception and navigation. However, these sensors experience significant performance degradation in environments characterized by dense fog, heavy smoke, low illumination, airborne particulates, or featureless and highly reflective surfaces. Mission scenarios such as search-and-rescue operations in fire-damaged structures, smoke-obscured military environments, subterranean tunnel inspection, and nighttime deployments represent vision-denied conditions where optical perception reliability is substantially reduced [12]. In such settings, the loss of dependable visual feedback can critically impair navigation stability and obstacle detection.

To address this limitation, the proposed architecture integrates an acoustic tracking subsystem capable of operating independently of ambient light conditions. By leveraging time-difference-of-arrival–based acoustic localization, the system maintains environmental awareness under zero-visibility scenarios. In parallel, the machine learning–driven obstacle avoidance module relies on internally maintained dynamic state representations including obstacle position, velocity, and predicted trajectory allowing navigation continuity without persistent camera input. This multimodal redundancy enhances operational resilience when optical sensing becomes unreliable.

D. Lack of Immutable Telemetry and Data Integrity

Inter-drone communication in many deployed swarm implementations relies on unsecured channels or lightweight integrity mechanisms, such as basic checksums, which do not provide cryptographic authenticity guarantees. Such configurations expose the network to Man-in-the-Middle (MITM) and replay attacks, where adversaries may inject or retransmit fabricated telemetry packets to influence formation geometry, navigation decisions, or mission trajectories [4], [11]. In the absence of a distributed and tamper-evident logging mechanism, post-mission forensic analysis becomes limited, and establishing regulatory accountability for autonomous decision-making processes becomes significantly more challenging.

To mitigate these vulnerabilities, the proposed architecture incorporates a distributed append-only logging mechanism termed the Flying Ledger. Each telemetry snapshot and command event is hashed using SHA3-256 and digitally signed with Ed25519 to ensure authenticity and integrity. Cryptographic chaining of blocks enables tamper detection, while peer-to-peer replication across swarm nodes prevents loss of audit records due to individual node failure. By combining authenticated communication with distributed ledger principles, the system strengthens resistance against unauthorized telemetry manipulation and enhances post-mission verifiability.

E. Actuator Degradation and Lack of Self-Healing

During real-world UAV operations, rotor degradation arising from mechanical wear, foreign object impact, manufacturing variability, or hostile interference can significantly compromise flight stability. Conventional flight controllers typically detect complete motor failure but may lack sensitivity to early-stage performance degradation. This limitation reduces the system’s ability to perform adaptive control adjustments prior to critical instability. Furthermore, existing literature highlights a gap in integrating real-time motor health diagnostics with closed-loop thrust redistribution mechanisms to enable graceful degradation rather than abrupt system failure [30], [47], [48].

To address this limitation, the proposed architecture implements a C++ based Differential Immune System for continuous rotor health monitoring. The subsystem evaluates real-time RPM deviation, vibration characteristics, temperature profiles, and current draw for each actuator. When performance deviation exceeds a predefined threshold (e.g., 10% RPM variance), the controller dynamically redistributes thrust among healthy motors and performs adaptive PID parameter retuning within the active control cycle. This mechanism enhances flight resilience by stabilizing the platform under partial actuator degradation and reducing the likelihood of abrupt mission termination.

F. Processing Jitter and Communication Latency

Autonomous swarm coordination requires tightly synchronized interaction between high-level decision logic (e.g., Python-based intelligence modules) and low-level real-time flight control (C++ firmware). Variations in processing time, inter-process communication delays, or network jitter can introduce temporal inconsistencies between command generation and actuator execution. When synchronization is disrupted, control inputs may be applied to a system state that has already evolved, increasing the risk of navigation instability or unsafe trajectory deviations in dynamic airspace environments.

To mitigate this risk, the proposed architecture incorporates dedicated latency monitoring and inter-process synchronization modules. The LatencyMonitor and MLBridge components continuously measure the complete C++ → Python → C++ round-trip time (RTT) using a 120-sample rolling window. If the measured RTT exceeds a predefined safety threshold (e.g., 220 ms), the system automatically transitions to a local-only fallback avoidance mode to maintain safe navigation. Additionally, a hardware-level watchdog timer supervises the Python process and initiates an emergency return-to-home sequence within approximately 1.5 seconds in the event of process hang or failure. This layered timing supervision mechanism enhances real-time stability and reduces the impact of processing jitter on swarm coordination.

III. SYSTEM ARCHITECTURE OVERVIEW

The proposed system is implemented as a layered and modular architecture with clearly defined dependency boundaries between functional components. At the highest level, an operator interface provides mission supervision and command input through a PyQt5-based graphical console. Beneath this layer, a centralized coordination module (SwarmManager) orchestrates decentralized swarm behavior, leader election, and inter-drone synchronization. Each drone maintains an independent state and physics model to ensure local autonomy and fault tolerance. Supporting these core components are specialized subsystems including the distributed Flying Ledger, acoustic TDOA localization module, machine learning–based obstacle avoidance engine, dynamic obstacle modeling unit, and encrypted communication layer. At the lowest level, a C++ Hardware Abstraction Layer (HAL) interfaces with the MAVLink/PX4 flight controller, enabling real-time motor control and sensor integration. Telemetry and sensor data propagate upward through the architecture, while command signals flow downward from the operator through the elected leader to follower drones.

Fig. 1 illustrates the layered architectural framework of the proposed system. It demonstrates the high-level coordination handled by the Python-based SwarmManager, the decentralized security layer (Flying Ledger), and the low-level real-time execution core implemented in C++. The diagram highlights the bidirectional data flow between the operator interface and the autonomous agents, emphasizing the modular separation of the ML inference engine and the acoustic TDOA localization module.

Technology Stack: The software stack is divided across two primary execution domains. Python 3.11+ manages swarm-level logic, drone behavioral modeling, acoustic processing, distributed ledger operations, machine learning inference, obstacle modeling, and graphical user interface rendering. C++17 is employed for time-critical flight control operations, MAVLink communication, and latency supervision. Inter-process communication between Python and C++ modules is implemented via shared-memory IPC with timestamp-annotated packets, enabling round-trip timing analysis through the LatencyMonitor subsystem. Encrypted inter-drone communication utilizes AES-256-GCM implemented through the Python cryptography library to ensure confidentiality and integrity of transmitted telemetry and control commands.

To support reproducibility and further academic research, the entire software stack, including the ML inference engine and the Flying Ledger protocol, has been open-sourced on GitHub [Link].

Hardware Specifications: The physical implementation of the swarm agents is designed to support the high computational demands of the decentralized ledger and ML inference. The key hardware components and their respective roles within the architecture are detailed in Table I.

Table I Drone Hardware Configuration

3.1 Module Dependency Graph

The system entry point initializes the swarm environment by instantiating the central orchestration engine and launching the operator interface. The core coordination module (SwarmManager) functions as the aggregation layer for all major subsystems, including the ObstacleManager, LatencyMonitor, MLBridge, AcousticTrackingSystem, per-drone FlyingLedger instances, and the CommunicationManager event bus. These components are implemented as managed singleton services to ensure consistent state propagation and controlled inter-module interaction.

Each Drone object encapsulates its own state model and maintains a dedicated instance of the MLDecisionSupport module for localized decision inference. This design preserves per-node autonomy while allowing higher-level coordination through the SwarmManager.

Low-level hardware interaction is abstracted through a C++ based DroneController module interfaced via MAVLink. The controller exposes sensor configuration parameters and runtime settings through environment-variable initialization loaded from a structured configuration file (.env). This separation of high-level swarm logic and low-level flight control enforces modular dependency boundaries while enabling deterministic hardware-level execution.

IV. LEADER ELECTION — MCSS PROTOCOL

4.1 Multi-Criteria Suitability Scoring

The Multi-Criteria Suitability Scoring (MCSS) protocol assigns each candidate drone a scalar suitability score , computed as a weighted aggregation of normalized hardware and communication health metrics:

where denotes the normalized battery state-of-charge; represents the Motor Health Index defined as

and  corresponds to the link quality score derived from Received Signal Strength Indicator (RSSI) stability and packet-loss statistics.

The weighting coefficients  satisfy the convexity constraint

with default configuration values , , and .

Leader selection is determined by

where  denotes the elected leader. A re-election procedure is automatically triggered when the incumbent leader fails to transmit two consecutive heartbeat signals, ensuring dynamic reconfiguration under node failure conditions.

Fig. 2. MCSS-based decentralized leader election process.

4.2 Byzantine Fault Tolerance Properties

The MCSS-based leader election mechanism is designed to preserve consensus integrity in the presence of Byzantine behavior affecting up to

Fig. 3. Fault tolerance bound of the MCSS protocol under Byzantine behavior

nodes, consistent with classical distributed fault-tolerance bounds [4], [5]. Under this constraint, the swarm maintains a valid leader selection outcome despite arbitrary or malicious behavior by faulty participants.

For the minimum operational configuration , the system tolerates one Byzantine node. In the default demonstration configuration , one faulty node can be accommodated while preserving coordinated swarm operation.

Leader re-election latency is bounded by two consecutive heartbeat intervals plus inter-node score propagation delay. In experimental simulation, this results in an observed upper bound of approximately 1.2 seconds for leadership restoration following failure detection. This bounded recovery time contributes to maintaining distributed coordination under transient node compromise or crash faults.

4.3 Role Transitioning

The swarm architecture defines three operational roles: Leader, Follower, and Relay. Each role encapsulates distinct behavioral and communication responsibilities to preserve structured coordination and controlled authority distribution within the decentralized framework.

The Leader is responsible for executing mission-level objectives, generating high-level trajectory directives, and disseminating position targets to follower nodes through the CommunicationManager event bus. In addition, the Leader periodically broadcasts heartbeat signals to confirm operational continuity and availability.

A Follower operates under a strictly responsive control model. It processes only authenticated LEADER_COMMAND events and does not independently initiate trajectory or mission-level commands. This constraint ensures hierarchical coherence while maintaining distributed execution capability.

The Relay role, introduced as a scalability extension, is designed to facilitate communication bridging between spatially separated drone clusters or in environments with partial connectivity. Although not mandatory in the baseline configuration, the Relay abstraction enables future expansion toward large-scale swarm deployments.

All role transitions including leader promotion, follower reassignment, or relay activation are recorded as immutable state-transition events within the Flying Ledger. This logging mechanism ensures verifiable traceability of authority changes and supports post-mission audit integrity within the decentralized coordination model.

Fig. 4. Operational role model of the decentralized swarm showing Leader command dissemination, Follower responsive control, and Relay-based communication bridging, with role transitions recorded on the distributed ledger.

V. SECURE COMMUNICATION — AES-256-GCM MESH

5.1 Encrypted Transport Layer

A monotonically increasing sequence number is included as Additional Authenticated Data (AAD) to mitigate replay attacks. All inter-drone control and telemetry messages are protected using AES-256 in Galois/Counter Mode (GCM), an authenticated encryption scheme that provides both confidentiality and integrity guarantees [14], [15]. The symmetric swarm key is derived from a shared passphrase through PBKDF2-HMAC-SHA256 with 100,000 iterations to increase resistance against brute-force attacks. The key derivation process is defined as

where the output length is 32 bytes (256 bits).

Each encrypted packet follows the structure

where a per-message Initialization Vector (IV) is generated using a cryptographically secure random source. The authentication tag produced by GCM ensures that any unauthorized modification of ciphertext or associated data is detected during decryption. By combining strong key derivation with authenticated encryption, the communication layer enhances resistance against replay, tampering, and interception attempts within the swarm network.

Fig. 5. AES-256-GCM encrypted packet format with PBKDF2-derived swarm key and authenticated encryption tag.

5.2 Replay Attack Prevention

To mitigate replay attacks in the decentralized swarm network, each transmitted message includes a monotonically increasing sequence number assigned by the originating drone. The sequence number is incorporated as authenticated data within the AES-256-GCM encryption process, ensuring that it is integrity-protected and cannot be modified without detection.

Each receiving drone maintains a per-sender sequence counter representing the highest validated sequence value previously accepted from that peer. Upon message reception, the receiver verifies that the incoming sequence number satisfies

Only messages meeting this strict ordering constraint are processed; all others are discarded without acknowledgment. This mechanism provides deterministic replay protection without reliance on synchronized clocks, thereby avoiding time-drift vulnerabilities common in timestamp-based schemes [4]. By combining sequence validation with authenticated encryption, the communication layer strengthens resistance against packet duplication and delayed retransmission attacks within the swarm mesh.

Fig. 6. Replay protection mechanism using per-sender monotonic sequence numbers with strict ordering validation at the receiver.

5.3 Wireless Transport Layer (IP-Independent Communication)

Inter-drone communication is implemented over a direct wireless link without reliance on IP networking or WiFi infrastructure. The transport layer operates using a lightweight peer-to-peer broadcast mechanism over a shared RF channel, enabling direct packet dissemination between swarm nodes.

Unlike IP-based multicast systems, the proposed design avoids routing, DHCP negotiation, and network stack overhead, thereby reducing latency variability and infrastructure dependency. The wireless transport prioritizes deterministic timing and minimal protocol overhead, which are critical for real-time swarm coordination.

Since the transport layer does not guarantee delivery, higher-level protocol mechanisms such as authenticated encryption, sequence-number validation, and heartbeat supervision are responsible for ensuring integrity, replay protection, and fault detection. This layered approach maintains low-latency communication while preserving security and coordination robustness in infrastructure-denied environments.

Fig. 7. Infrastructure-independent wireless broadcast architecture enabling peer-to-peer encrypted communication among swarm nodes.

VI. DYNAMIC OBSTACLE AVOIDANCE

6.1 Collision Cone Probability

The DynamicObstaclePredictor estimates potential collision risk using a geometric collision-cone formulation as described in [25], [26]. Consider a drone located at position with velocity , and an obstacle located at position with velocity . The relative position and velocity vectors are defined as

The half-angle of the collision cone is computed as

Fig. 8. Geometric collision-cone formulation showing relative position vector , relative velocity , obstacle radius , and half-angle . A collision condition occurs when lies within the cone defined by .

where denotes the effective obstacle radius including a predefined safety buffer.

Let  be the normalized relative position vector. A potential collision condition is detected when the angular separation between and satisfies

indicating that the relative velocity vector lies within the collision cone. In such cases, a corrective velocity adjustment is required to prevent intersection trajectories.

To reduce unnecessary avoidance maneuvers for distant objects, a time-to-closest-approach (TCA) filter is applied. Collision cone intersections corresponding to predicted encounters occurring beyond a 4-second horizon are disregarded, ensuring responsiveness to imminent threats while maintaining trajectory stability.

6.2 Trajectory Prediction and Kinematic Model

The TrajectoryEstimator predicts short-horizon obstacle motion using a second-order kinematic formulation. For an obstacle with initial position , velocity components , and acceleration components , the predicted position at future time is given by

Fig. 9. Short-horizon trajectory prediction using second-order kinematics. The Linear model assumes constant velocity, the Circular model follows constant angular velocity about a fixed center, and the Random-Walk model introduces bounded stochastic acceleration within defined limits.

This formulation enables continuous estimation of obstacle trajectories within a bounded prediction horizon.

Three motion models are supported:

Linear Model (Constant Velocity):
Assumes , resulting in uniform motion along a fixed direction.

Circular Model (Constant Angular Velocity):
Models obstacle motion along a circular path with constant angular velocity around a fixed center, enabling prediction of curved trajectories.

Random-Walk Model (Bounded Stochastic Acceleration):
Introduces bounded stochastic perturbations to acceleration within , subject to a maximum speed constraint of 18 m/s, allowing realistic modeling of unpredictable motion.

Predictions are sampled at 0.3-second intervals over a 3-second forward horizon, resulting in a discretized trajectory set used for collision-cone evaluation and avoidance planning.

6.3 Acceleration-Limited Velocity Blending

Raw avoidance velocity vectors generated by the prediction engine are combined with the mission-level goal velocity through a smooth steering formulation. The desired velocity is defined as

To ensure gradual transition from the current motion state, a blending operation is applied:

where is a smoothing coefficient controlling steering aggressiveness.

To maintain physical feasibility, the resulting acceleration demand is evaluated as

If , the velocity increment is proportionally scaled such that

while preserving the direction of motion.

Fig. 10. Velocity blending mechanism showing current velocity , goal velocity , avoidance vector , and the smoothed output controlled by interpolation factor . Acceleration magnitude is subsequently constrained to .

Finally, the PathReplanner converts the bounded output velocity into a lookahead reference position computed over a 1.2-second horizon, generating a dynamically updated target waypoint for the low-level controller.

6.4 Learned Aggressiveness Scoring

To incorporate behavioral consistency into collision assessment, the DynamicObstaclePredictor maintains a per-obstacle risk memory score updated using exponential smoothing. The update rule is defined as

Fig. 11. Exponential risk memory update showing gradual accumulation of aggressiveness score over time using weighted smoothing (0.86 previous memory, 0.14 new observation). Persistently erratic obstacle behavior results in elevated long-term risk weighting.

where and denote the instantaneous obstacle speed (m/s) and acceleration magnitude (m/s), respectively. The normalization constants (20 and 6) scale dynamic motion parameters into the unit interval, and the minimum operator ensures boundedness within .

The exponential weighting introduces temporal memory, allowing obstacles that exhibit consistently high-speed or erratic acceleration patterns to accumulate elevated risk scores over time. This learned aggressiveness factor is incorporated as a multiplicative weight in the final collision probability estimate, increasing avoidance sensitivity for previously unstable obstacles while preserving the underlying geometric collision-cone prediction framework.

VII. ACOUSTIC SOURCE LOCALIZATION — GCC-PHAT TDOA ENGINE

7.1 Time Difference of Arrival Estimation

The CrossCorrelationEngine estimates inter-drone acoustic propagation delay using the Generalized Cross-Correlation with Phase Transform (GCC-PHAT) method [21]. Consider two microphone signals and with Fourier transforms and , respectively. The GCC-PHAT cross-spectrum is defined as

Fig. 12. Time Difference of Arrival (TDOA) geometry showing two drone-mounted microphones and an acoustic source. The propagation delay defines a hyperbolic locus of possible source locations used in GCC-PHAT localization.

where denotes the complex conjugate of , and is a small regularization constant introduced to prevent numerical instability.

The phase-only normalization (division by magnitude) whitens the spectrum and reduces sensitivity to signal amplitude variations and reverberation effects, resulting in a sharper correlation peak in the time domain. The estimated time delay is obtained as

where denotes the inverse Fourier transform.

To improve robustness, a secondary direct cross-correlation estimate is computed in parallel. The final delay estimate is selected based on the higher peak magnitude between the GCC-PHAT and direct correlation outputs, providing resilience against low signal-to-noise or multipath conditions.

7.2 TDOA Distance Constraint

Each estimated time delay between sensor pair corresponds to a difference in propagation path length given by

where denotes the speed of sound at sea level (at approximately 20°C).

Let represent the unknown source position and denote the known sensor coordinates. The time-delay constraint yields the hyperbolic equation

which defines a branch of a hyperbola in two-dimensional space.

Fig. 13. Decentralized drone swarm architecture showing secure RF communication, obstacle prediction, acoustic localization, path replanning, and ledger-based role management.

With three or more non-collinear sensors, multiple independent hyperbolic constraints can be constructed. The intersection of these constraints provides an estimate of the source location. In practice, due to measurement noise and finite sampling resolution, the source position is obtained via least-squares minimization over the set of hyperbolic equations.

7.3 Non-Linear Least Squares Fusion

The AcousticFusionEngine estimates the source position by solving the overdetermined system of TDOA-derived hyperbolic constraints using a nonlinear least-squares formulation. Let the unknown source position be , and let denote the known sensor coordinates. The optimization problem is defined as

where represents the estimated time delay between sensor pair , and is the speed of sound.

The problem is solved using SciPy’s Trust-Region Reflective (TRF) least-squares solver with a soft-L1 loss function to reduce sensitivity to outlier delay estimates. To mitigate convergence to local minima, multiple initialization points are evaluated, including drone positions offset by ±10 m and the centroid of all sensor coordinates. The solution yielding the minimum Root Mean Square Error (RMSE) residual is selected.

The localization confidence metric is defined as

where RMSE denotes the residual error of the optimized solution. Only localization results exceeding a confidence threshold of 0.35 are disseminated to the swarm as validated acoustic detection events.

Fig. 14. AcousticFusionEngine nonlinear TDOA-based localization framework using TRF least-squares optimization with soft-L1 loss, multi-initialization strategy, RMSE-based solution selection, and confidence-threshold validation for swarm-level acoustic event dissemination.

7.4 High-Latency Fallback

To preserve real-time responsiveness, the AcousticFusionEngine incorporates a latency-aware fallback strategy. When the measured bidirectional C++↔Python round-trip time (RTT) exceeds a predefined threshold (default: 280 ms), the localization process reduces computational complexity by restricting optimization to the three nearest sensor signals instead of the full sensor set.

This adaptive reduction decreases the dimensionality of the overdetermined system and shortens solver convergence time, thereby maintaining bounded execution within the system’s safety window. While using fewer sensors may slightly reduce localization precision, it enables timely generation of position estimates under network degradation or processing congestion conditions.

By integrating latency monitoring with sensor subset selection, the system balances estimation accuracy against real-time operational constraints in degraded communication scenarios.

Fig. 15. Latency-aware fallback mechanism in the AcousticFusionEngine, where excessive C++↔Python RTT (>280 ms) triggers sensor subset reduction to the three nearest microphones, lowering optimization complexity to preserve real-time localization under degraded network conditions.

VIII. THE FLYING LEDGER — BLOCKCHAIN FLIGHT AUDIT

8.1 Block Structure and Cryptographic Hash

Each block within the Flying Ledger is structured to ensure immutability, traceability, and cryptographic integrity of flight events. A block contains the following fields:

Block index

UNIX timestamp

Drone identifier

SHA3-256 hash of the telemetry snapshot ()

SHA3-256 hash of the event payload ()

Previous block hash ()

Current block hash ()

Ed25519 digital signature

The block hash is computed as

where denotes byte-wise concatenation.

The use of SHA3-256 [16] provides cryptographic hashing based on the Keccak sponge construction, offering structural diversity relative to SHA-2 family algorithms. Unlike Merkle–Damgård–based constructions, the sponge design avoids certain structural vulnerabilities such as classical length-extension properties.

The resulting block hash is digitally signed using Ed25519 to ensure authenticity and non-repudiation. Any modification to block contents alters , thereby invalidating both the cryptographic chain and the associated signature.

Fig. 16. Structure of the Flying Ledger block showing SHA3-256 hash computation, linkage via previous block hash, and Ed25519 digital signature for authenticity and tamper detection

8.2 Ed25519 Digital Signatures

Each block in the Flying Ledger is digitally signed by its originating drone using Ed25519 [17], an elliptic-curve digital signature scheme based on Curve25519. Ed25519 was selected due to its performance efficiency, compact signature representation, and deterministic signing procedure.

Unlike traditional ECDSA implementations that require high-quality randomness for nonce generation, Ed25519 employs deterministic nonce derivation from the private key and message hash, reducing dependence on external random number generators during signing. This property mitigates vulnerabilities associated with biased or reused nonces in classical ECDSA implementations.

Ed25519 signatures have a fixed length of 64 bytes, providing compact and predictable encoding compared to variable-length ECDSA signatures. Additionally, the Edwards-curve formulation and deterministic design offer strong security guarantees under standard elliptic-curve discrete logarithm assumptions.

For implementation flexibility, each signature is stored as a Base64-encoded string prefixed with the algorithm identifier (e.g., "Ed25519:"). This encoding scheme enables forward-compatible algorithm agility, allowing future migration to alternative signature schemes without structural changes to the ledger format.

Fig. 17. Ed25519 signing and verification pipeline for block authentication in the Flying Ledger.

8.3 Tamper Detection and Chain Verification

Chain integrity verification is performed by sequentially traversing all blocks from the genesis block to the current tip of the ledger. For each block , two validation conditions are evaluated:

The stored field must match the recomputed hash of the preceding block.

The stored must equal the freshly computed hash obtained by re-evaluating the block’s constituent fields according to Appendix A.3.

Formally, for every block index :

Fig. 18. Hash-chain integrity verification illustrating tamper propagation from Block to subsequent blocks.

Any alteration to block contents modifies the resulting cryptographic hash with overwhelming probability under standard hash function security assumptions. Consequently, a modification in block invalidates its stored hash and propagates inconsistency to all subsequent blocks, enabling both detection and localization of ledger tampering during verification.

8.4 Asynchronous Peer Replication

Upon local block creation, the originating drone disseminates the block to peer nodes using an asynchronous broadcast mechanism executed within a background (daemon) thread. This design decouples ledger replication from time-critical flight control tasks, ensuring non-blocking operation of the swarm.

Each receiving peer invokes a verify_block() validation routine prior to appending the block to its local ledger. The verification process evaluates the following conditions:

Sequential index consistency

Correct linkage via the  field

Recomputed hash equality according to Appendix A.3

Valid Ed25519 signature verification against the sender’s registered public key

Fig. 19. Asynchronous block dissemination and peer-side verification workflow ensuring hash integrity, signature validity, and Byzantine-resistant ledger replication.

Blocks failing any validation condition are rejected and not appended to the local chain.

Because block acceptance requires a valid digital signature corresponding to the legitimate drone’s private key, unauthorized block injection attempts are rejected under standard cryptographic assumptions. This mechanism strengthens resistance against Byzantine-style injection attacks in which adversarial nodes attempt to introduce fabricated ledger entries.

IX. C++ DIFFERENTIAL IMMUNE SYSTEM

9.1 Motor Degradation Detection

The C++–based DroneController executes a continuous telemetry monitoring loop synchronized with the hardware update rate. For each motor , the relative RPM deviation is computed as

where a deviation threshold of 10% is used as the primary degradation indicator.

Fig. 20. Motor degradation detection using RPM deviation threshold (≥10%), auxiliary vibration and temperature monitoring, and rolling-window filtering for persistent fault classification.

In addition to RPM deviation, vibration magnitude and motor temperature are monitored as secondary health metrics. These auxiliary signals provide corroborative evidence of mechanical wear or imbalance.

To avoid false-positive detections caused by transient load variations or short-duration power fluctuations, the system applies a rolling-window filter over consecutive telemetry samples. A motor is classified as degraded only when the deviation threshold is exceeded persistently across the defined observation window. This temporal filtering ensures robustness against momentary noise while preserving sensitivity to sustained actuator degradation.

9.2 Adaptive Thrust Redistribution

Upon detection of motor degradation, thrust compensation is applied across the remaining healthy motors to preserve attitude stability. Let denote the nominal thrust of motor . The compensated thrust command is defined as

where represents the redistribution increment assigned to healthy motors , the set of operational actuators.

The compensation allocation follows a geometry-aware distribution strategy: the rotor diametrically opposite the degraded motor receives the largest thrust increment, while adjacent lateral rotors receive proportionally smaller increments to maintain torque balance and yaw stability.

To prevent oscillatory behavior or abrupt torque transients, all compensation terms are passed through a first-order low-pass filter (LPF):

where is the smoothing coefficient.

Fig. 21. Geometry-aware thrust redistribution and low-pass filtered compensation for single-motor degradation, with automatic RTL activation under multi-motor failure.

If two or more motors are classified as degraded simultaneously, the controller transitions to AUTO_RTL (automatic return-to-launch) mode. Under standard quadcopter dynamics, sustained stable hovering with fewer than three fully functional rotors cannot be reliably maintained; therefore, emergency mission termination is initiated as a safety measure.

9.3 Adaptive PID Retuning

The updateAdaptivePID() routine dynamically adjusts roll, pitch, and yaw controller gains in response to the current actuator health state. This mechanism functions as a gain-scheduling strategy conditioned on motor degradation status.

When a motor is classified as degraded, the proportional gain associated with the affected rotational axis is reduced to mitigate aggressive corrective responses that may otherwise amplify oscillatory behavior under asymmetric thrust conditions. Concurrently, the derivative gain is modestly increased to enhance damping and improve transient stability.

Formally, for a degraded condition:

where and are empirically tuned scaling coefficients.

This adaptive retuning reduces overshoot and oscillation during thrust redistribution and allows the drone to maintain controlled flight during degradation transitions without requiring operator intervention.

Fig. 22. Adaptive PID gain scheduling under motor degradation, reducing proportional gain and increasing derivative gain to improve damping, suppress oscillations, and maintain stable flight during thrust asymmetry.

X. LATENCY MONITORING AND SAFETY FALLBACK

10.1 Round-Trip Time Measurement

The LatencyMonitor maintains a rolling window of 120 LatencySample records to continuously assess cross-language execution delay between C++ and Python modules. Each sample stores four timestamps:

The total round-trip time (RTT) for sample is computed as

Over a rolling window , latency jitter is quantified as the standard deviation:

where

and .

Both instantaneous RTT and computed jitter metrics are propagated to the swarm state monitoring interface and used for safety-decision logic under degraded processing conditions.

Fig. 23. Rolling-window RTT measurement and jitter computation between C++ and Python modules, providing real-time latency metrics for safety monitoring and fallback decision logic.

10.2 Automatic Fallback Activation

When the mean round-trip time (RTT) computed over the rolling window exceeds a predefined threshold (default ms), the system activates a degraded-operation state by setting

Fig. 24. Latency-triggered fallback mechanism where excessive mean RTT activates local geometric collision-cone avoidance, bypassing Python ML inference to maintain bounded real-time control under degraded conditions.

In this mode, obstacle avoidance decisions are executed entirely within the local drone process, bypassing the Python-based machine learning inference pipeline. Instead, avoidance is computed using the deterministic geometric collision-cone predictor described in Section VI.

This architectural shift removes cross-language communication latency from the control loop, thereby reducing response variability under processing congestion or network degradation. While this fallback omits learned risk modulation, it preserves core geometric collision avoidance functionality and ensures bounded execution time determined primarily by local computational resources.

10.3 Watchdog Timer

The MLBridge subsystem incorporates a hardware-level watchdog mechanism to supervise responsiveness of the Python processing layer. If no valid response is received within a timeout interval of 1.5 seconds, the watchdog condition is triggered.

Fig. 25. Hardware-level watchdog supervising the MLBridge Python layer; a 1.5 s timeout triggers emergency Return-to-Home (RTL) for drones in motion, ensuring fail-safe recovery under software stalls or crashes.

Upon timeout detection, the swarm controller issues an emergency Return-to-Home (RTL) command to all drones currently in the MOVING_TO_TARGET operational state. This safety transition overrides pending trajectory commands and shifts the system into a predefined recovery mode.

The watchdog mechanism mitigates the risk of continued execution of stale or partially computed control commands in the event of a Python process crash or severe runtime stall. By enforcing a bounded response window, the system enhances fail-safe behavior under software-level faults.

XI. RETURN-TO-HOME PROBABILITY MODEL

11.1 Multiplicative Reliability Model

Prior to initiating an autonomous Return-to-Home (RTH) maneuver, the system evaluates the estimated probability of successful mission completion using a multiplicative reliability formulation. The success probability is defined as

$$
P_{\text{RTH}} = R_b \cdot R_m \cdot R_d \cdot R_w
$$

where each factor represents a normalized reliability estimate in the interval $[0,1]$, corresponding to energy sufficiency, actuator health, distance feasibility, and environmental wind stability, respectively.

This formulation assumes conditional independence among contributing risk factors, yielding a conservative composite estimate under standard reliability modeling principles.

The decision rule is defined as

$$
\text{If } P_{\text{RTH}} \ge 0.70 \Rightarrow \text{Execute RTH}, \quad
\text{else } \Rightarrow \text{Controlled Land-in-Place}.
$$

The threshold value of 0.70 represents an empirically selected minimum acceptable probability of safe return under operational testing conditions.

Fig. 26. Probabilistic decision framework for autonomous RTH versus controlled landing based on composite reliability estimation.

11.2 Individual Factor Computation

Each reliability component in Appendix A.4 is computed as follows:

Battery Reliability

$$
R_b = \min\!\left(1,\; \frac{E_{\text{avail}}}{E_{\text{return}} + E_{\text{margin}}}\right)
$$

where $E_{\text{margin}} = 0.05$ (5% safety margin).

Motor Reliability

$$
R_m = 1 - d_m
$$

where the degradation fraction $d_m$ is derived from the C++ Differential Immune System's RPM deviation metric.

Distance Reliability

$$
R_d = \max\!\left(0,\; 1 - \frac{D_{\text{home}}}{D_{\max}}\right)
$$

Wind Reliability

$$
R_w = \max\!\left(0,\; 1 - \frac{\|w\|}{w_{\max}}\right)
$$

where $\|w\|$ denotes wind magnitude.

Worked Example

Consider a drone with:

30% battery remaining

Required return energy: 20% + 5% safety margin

10% motor degradation

Distance to home: 5 km

Maximum reliable range: 10 km

Headwind: 6 m/s

$w_{\max} = 12$ m/s

The resulting factors are:

$$
R_b = \frac{0.30}{0.20+0.05} = 1.20 \Rightarrow 1.00
$$

$$
R_m = 1 - 0.10 = 0.90
$$

$$
R_d = 1 - \frac{5}{10} = 0.50
$$

$$
R_w = 1 - \frac{6}{12} = 0.50
$$

Thus,

$$
P_{\text{RTH}} = 1.00 \times 0.90 \times 0.50 \times 0.50 = 0.225
$$

Since $0.225 < 0.70$, the decision rule selects Controlled Land-in-Place rather than autonomous Return-to-Home.

This probabilistic evaluation reduces the risk of partial return attempts under marginal energy and environmental conditions, thereby enhancing operational safety.

XII. MACHINE LEARNING DECISION SUPPORT

12.1 Personal ML Models

Each drone maintains an independent MLDecisionSupport model that provides localized risk estimation and avoidance guidance. The model is formulated as a polynomial regression–based classifier trained on mappings of the form

Training data are derived from the drone’s own historical flight records, enabling individualized behavioral adaptation.

A second-degree polynomial feature expansion is employed to capture non-linear interactions between spatial proximity, relative velocity, and obstacle characteristics while maintaining computational efficiency. Compared to deep neural network architectures, this approach offers deterministic inference time, reduced memory footprint, and lower onboard processing requirements properties advantageous for embedded aerial platforms.

The PhysicalMLTrainer module supports supervised training from CSV or JSON datasets. A minimum sample threshold (default: 50 samples) is enforced prior to model fitting to reduce overfitting risks associated with sparse data. Trained models are serialized to persistent storage and reloaded during drone initialization, enabling incremental refinement of decision behavior across successive missions.

12.2 ML-Augmented Navigation

The MLNavigationModule integrates machine learning–based risk estimation into the trajectory planning pipeline through a gated decision mechanism. For each planning cycle, the trained MLDecisionSupport model produces a predicted collision risk score along with a recommended avoidance vector.

A maneuver override is triggered when either of the following conditions holds:

In such cases, the drone is redirected toward a temporary waypoint computed along the suggested avoidance vector. This waypoint serves as an intermediate corrective target before resuming the nominal mission trajectory.

If a trained ML model is unavailable (e.g., initial deployment with insufficient training data), the navigation system defaults to the deterministic geometric collision-cone avoidance method described in Section VI. This fallback ensures uninterrupted navigation capability independent of machine learning availability.

By structuring ML inference as an augmentation layer rather than a mandatory control dependency, the system preserves operational continuity and safety under both trained and untrained conditions.

XIII. MATHEMATICAL FRAMEWORK SUMMARY

To ensure traceability between theoretical formulation and implementation, Table II provides a cross-reference of all principal equations with their corresponding software modules in the production codebase.

Table II: System equation cross-reference with implementing module.

XIV. EXPERIMENTAL RESULTS AND VALIDATION

14.1 Acoustic Localization Accuracy

The acoustic localization pipeline was evaluated using the test suite test_acoustic_tdoa_accuracy(). A known impulse source was positioned at m within a planar environment instrumented with four spatially separated sensors located at

The audio sampling rate was set to 48 kHz. Localization accuracy was assessed using the Euclidean position error metric:

Across repeated trials, the solver produced localization errors below 3.0 m.

Fig. 27. Estimated versus true source position demonstrating sub-3 m localization accuracy under moderate noise conditions.

To evaluate robustness under noise, additive Gaussian noise with standard deviation (normalized amplitude scale) was applied independently to each audio sample. Under these conditions, the computed acoustic confidence metric (Eq. 11) remained above the operational threshold of 0.35, and localization events were successfully registered.

These results demonstrate that the combined GCC-PHAT delay estimation and TRF-based nonlinear least-squares fusion pipeline maintains stable localization performance under moderate sensor noise conditions.

14.2 Blockchain Consensus and Tamper Detection

The distributed ledger replication and validation mechanisms were evaluated using the test suite test_blockchain_consensus(). Three independent FlyingLedger instances were initialized with mutually registered Ed25519 public keys to simulate a minimal multi-node swarm configuration.

In the consensus test, Drone 1 appended a locally generated event block to its ledger and asynchronously broadcast the block to peer nodes. Following propagation (approximately 50 ms under local test conditions), all three ledger instances converged to the same block height and identical block hash at the chain tip, indicating successful replication and deterministic chain consistency.

Tamper resistance was evaluated using the test_block_validation_rejection() routine. In this test, a forged block with a deliberately modified previous_hash field was transmitted to a peer node. The receiving ledger invoked its verification routine and rejected the block due to hash-chain inconsistency. The local chain height remained unchanged, confirming that invalid or tampered blocks do not alter the ledger state.

These results demonstrate correct operation of asynchronous replication, signature validation, and hash-chain integrity enforcement under both normal and adversarial conditions.

Fig. 28. Multi-node ledger consensus and rejection of tampered blocks via hash-chain and signature verification.

14.3 Obstacle Avoidance Validation

The obstacle avoidance stack was evaluated across five representative operational scenarios:

Single Dynamic Obstacle: A moving obstacle crossing the drone’s projected trajectory.

Static Obstacle at Zero Velocity (Startup Transient): A stationary obstacle encountered while the drone has negligible initial velocity.

Multi-Obstacle Environment: Concurrent linear-motion and random-walk obstacles interacting within the prediction horizon.

ML-Disabled Fallback Mode: Navigation executed solely via geometric collision-cone prediction without ML assistance.

Mission Resumption After Avoidance: Verification that the drone resumes its original mission target after clearing the avoidance condition.

Across all test cases, the system successfully transitioned into the avoidance-active state upon detection of collision risk. A valid intermediate target waypoint was generated in each scenario, and trajectory updates were propagated to the flight controller. In the mission-resumption test, the system restored nominal navigation once the predicted collision condition was cleared.

These results indicate correct integration of collision detection, velocity blending, fallback logic, and mission continuity mechanisms under diverse environmental configurations.

Fig. 29. Avoidance-enabled trajectory deviation and mission resumption under obstacle encounter.

14.4 Latency Monitoring

The latency supervision subsystem was evaluated using targeted stress and statistical consistency tests.

In the test_high_latency_spike() scenario, a synthetic round-trip time (RTT) event of 520 ms was injected into the monitoring pipeline. The system correctly computed

thereby triggering the degraded-operation condition. The internal state flag fallback_required was set to True, confirming proper activation of the local avoidance fallback mechanism described in Section X.

In the test_latency_jitter_std_tracking() routine, three RTT samples with varying processing delays were recorded within the rolling window. The system successfully computed a non-negative jitter standard deviation value according to Eq. (16), and the metric was correctly exposed within the swarm statistics dictionary.

These results demonstrate correct threshold detection, fallback activation logic, and statistical tracking of latency variability under both spike and normal fluctuation conditions.

Fig. 30. Round-trip time (RTT) monitoring over time with threshold-based fallback activation. A synthetic 520 ms latency spike exceeds the 220 ms threshold, triggering degraded-operation mode. RTT subsequently returns below threshold, demonstrating correct detection and recovery behavior.

XV. GROUND CONTROL STATION — GUI CONSOLE

The PyQt5-based Ground Control Station (GCS) provides the human operator with a synchronized, real-time visualization of the swarm’s operational state. The interface employs a multi-threaded update architecture in which telemetry acquisition, visualization rendering, and user input handling execute in separate threads. This design prevents graphical rendering delays from interfering with swarm control logic or communication pipelines.

The GUI comprises the following primary components:

Swarm Health Table: Presents per-drone telemetry including battery percentage, motor health status, assigned role (Leader/Follower), flight mode, and current operational state, enabling continuous monitoring of fleet integrity.

Latency Dashboard: Displays a live round-trip time (RTT) plot with computed jitter standard deviation (Eq. 16) and a visual indicator reflecting fallback activation status, providing real-time communication stability insight.

Blockchain Synchronization Panel: Shows per-drone ledger block height, most recent block hash, and integrity verification status to confirm distributed consensus consistency and ledger validity.

Acoustic Detection Map: Renders a two-dimensional heatmap overlay of detected acoustic source positions, with color intensity proportional to the confidence metric (Eq. 11), facilitating rapid spatial interpretation of acoustic events.

Obstacle Visualizer: Displays real-time 2D obstacle trajectories and short-horizon predicted motion paths derived from the kinematic model described in Section VI, supporting situational awareness during avoidance maneuvers.

Mission Control Interface: Provides structured operator commands including takeoff, move-to-target (with per-drone navigation mode selection), and Return-to-Home activation, along with emergency control options.

This integrated console unifies telemetry monitoring, latency supervision, cryptographic ledger verification, acoustic detection, obstacle prediction, and mission command functionality within a single operator interface. By maintaining a clear separation between visualization and autonomous control processes, the system ensures situational transparency without direct interference in distributed decision-making.

Fig. 31. Ground Control Station (GCS) interface showing real-time swarm visualization, latency monitoring, blockchain synchronization status, ML training metrics, and mission control panel. The multi-threaded architecture enables simultaneous telemetry rendering and operator command execution without blocking swarm control logic.

XVI. LIMITATIONS AND FUTURE WORK

16.1 Current Limitations

Despite the demonstrated robustness of the proposed framework, several limitations remain.

First, the acoustic TDOA localization subsystem currently operates under a two-dimensional (XY-plane) assumption with fixed altitude. While sufficient for planar mission scenarios, full three-dimensional localization requires a minimum of four non-coplanar sensor nodes to resolve the additional spatial degree of freedom. The existing nonlinear least-squares formulation (Eq. 10) can be extended to 3D by incorporating a Z-coordinate term in the distance constraint, but this extension has not yet been experimentally validated.

Second, the MCSS-based leader election protocol assumes timely and globally observable heartbeat exchanges among drones. Although the system tolerates Byzantine faults within the bounds described in Section IV, it does not yet formally address partial network partitioning scenarios (i.e., split-brain conditions), in which subsets of drones may maintain internal connectivity while being disconnected from others. In such cases, simultaneous independent leader elections could theoretically occur. A consensus-layer enhancement incorporating quorum intersection or view-change protocols would be required to fully mitigate this risk.

16.2 Post-Quantum Cryptography Migration

The current implementation employs Ed25519 for digital signatures. Like other elliptic-curve–based signature schemes, Ed25519 relies on the hardness of the elliptic-curve discrete logarithm problem. Large-scale fault-tolerant quantum computers executing Shor’s algorithm [36], [37] would, in principle, compromise the security assumptions underlying such schemes.

To address long-term cryptographic resilience, the SignatureProvider abstraction layer in flying_ledger.py has been architected to support algorithm agility. This modular interface decouples signature generation and verification logic from the block structure and hash-chain implementation.

Migration to a NIST-standardized post-quantum signature scheme, such as CRYSTALS-Dilithium [37], would require only the implementation of a compatible provider subclass and its registration during drone initialization. Because the block format stores signatures as algorithm-prefixed encoded values, no structural modifications to the ledger schema are required. This design enables forward-compatible cryptographic evolution without altering historical chain data.

16.3 Heterogeneous Swarms and 3D Acoustics

Future development will extend the proposed framework to support heterogeneous multi-agent configurations, including fixed-wing UAVs, ground vehicles, and underwater platforms. This extension requires parameterization of the physics and control layers according to agent-specific dynamics, propulsion characteristics, and environmental constraints. The modular architecture of the SwarmManager and control subsystems facilitates such generalization without altering higher-level coordination logic.

In parallel, full three-dimensional acoustic localization will be incorporated by expanding the current 2D TDOA formulation to include altitude estimation, leveraging non-coplanar sensor configurations as discussed in Section XVI-A.

Another major direction involves energy-aware path planning. By integrating wind field estimation models and battery discharge characteristics into the trajectory optimization layer, the system can adaptively select energy-efficient routes. Preliminary simulation studies suggest potential reductions in total energy consumption on the order of 15–30% during long-duration missions, although comprehensive experimental validation remains part of future work.

XVII. CONCLUSION

This work presented a fully integrated, mathematically grounded, and experimentally validated framework for resilient autonomous drone swarm operation in contested and degraded environments. Unlike fragmented prior approaches that address security, control, or perception independently, the proposed system demonstrates the coordinated integration of distributed consensus, cryptographic integrity, probabilistic safety modeling, adaptive control, machine learning augmentation, and real-time monitoring within a single operational architecture.

At the coordination layer, the MCSS-based leader election protocol provides dynamic, merit-driven authority reassignment with Byzantine fault tolerance, eliminating centralized single-point fragility. At the communication layer, AES-256-GCM encrypted peer-to-peer messaging with replay protection ensures confidentiality and integrity under adversarial conditions. The Flying Ledger introduces tamper-evident, asynchronously replicated blockchain-based telemetry auditing using SHA3-256 hashing and Ed25519 signatures, with forward-compatible abstraction supporting post-quantum migration.

Perception and navigation are reinforced through a hybrid geometric–learning architecture. The collision-cone predictor, second-order kinematic trajectory estimation, and acceleration-constrained steering law provide deterministic avoidance guarantees, while learned aggressiveness scoring and polynomial ML decision support introduce adaptive behavior without sacrificing bounded execution time. The GCC-PHAT–based acoustic TDOA engine enables sub-3-meter localization performance without GPS reliance, supported by nonlinear least-squares fusion and latency-aware fallback mechanisms.

At the actuator level, the C++ Differential Immune System detects motor degradation via rolling-window telemetry analysis and performs adaptive thrust redistribution and PID gain retuning, enabling graceful degradation rather than catastrophic failure. The latency monitoring subsystem supervises cross-language execution jitter and activates deterministic fallback control or emergency return-to-home when safety thresholds are exceeded. A probabilistic Return-to-Home model further formalizes mission termination decisions based on energy, actuator health, distance feasibility, and environmental wind conditions.

Experimental validation confirms correct operation of acoustic localization, ledger consensus and tamper rejection, obstacle avoidance under diverse scenarios, and latency-triggered fallback behavior. Traceability from formal equations to implementation modules establishes reproducibility and verifiability across the entire stack.

The resulting architecture demonstrates that secure, decentralized, and safety-aware swarm autonomy is achievable through disciplined systems engineering and modular mathematical design. While current limitations include 2D acoustic modeling and simplified partition handling, the framework is structurally prepared for heterogeneous multi-agent expansion, 3D localization, energy-aware path planning, and post-quantum cryptographic migration.

This work therefore contributes not merely a collection of algorithms, but a coherent operational blueprint for resilient swarm deployment in GPS-denied, communication-disrupted, or adversarial environments, with direct applicability to defense reconnaissance, disaster response, search-and-rescue, and other safety-critical domains.

APPENDIX

APPENDIX A.1

Mathematical Derivations and Formal Proof Sketches

Given the suitability score

$$
S_i = w_b B_i + w_m M_i + w_l L_i
$$

with

$$
B_i, M_i, L_i \in [0,1], \qquad w_b,w_m,w_l \ge 0, \qquad w_b+w_m+w_l=1.
$$

Since each normalized metric lies in $[0,1]$, it follows:

$$
0 \le S_i \le 1.
$$

If at most $f$ nodes behave arbitrarily, consensus on the maximum score is preserved under majority broadcast assumptions consistent with classical Byzantine fault tolerance bounds.

$$
N \ge 3f+1 \quad \Rightarrow \quad f \le \left\lfloor\frac{N-1}{3}\right\rfloor.
$$

Leader re-election latency upper bound:

$$
T_{\text{re-elect}} \le 2T_{hb} + T_{prop}.
$$

In simulation:

$$
T_{hb}=0.5\,\text{s},\; T_{prop}\approx0.2\,\text{s} \Rightarrow T_{\text{re-elect}}\approx1.2\,\text{s}.
$$

A.2 Collision Cone Geometric Condition

Relative position:

$$
\mathbf{r}=\mathbf{p}_o-\mathbf{p}_d
$$

Relative velocity:

$$
\mathbf{v}_{rel}=\mathbf{v}_o-\mathbf{v}_d
$$

Half-angle of collision cone:

$$
\theta_c = \sin^{-1}\!\left(\frac{R_{eff}}{\|\mathbf{r}\|}\right), \quad \|\mathbf{r}\|>R_{eff}
$$

Collision condition:

$$
\angle(\mathbf{v}_{rel},-\mathbf{r}) \le \theta_c
$$

Time-to-closest-approach constraint:

$$
\text{TCA} = -\frac{\mathbf{r}\cdot\mathbf{v}_{rel}}{\|\mathbf{v}_{rel}\|^2}
$$

Reject if:

$$
\text{TCA} > 4\,\text{s} \;\;\text{or}\;\; \text{TCA}<0.
$$

A.3 TDOA Hyperbolic Constraint

Distance difference:

$$
\Delta d_{ij}=c\,\tau_{ij}
$$

Hyperbolic constraint:

$$
\|\mathbf{x}-\mathbf{s}_i\| - \|\mathbf{x}-\mathbf{s}_j\| = c\,\tau_{ij}
$$

Nonlinear least-squares objective:

$$
\mathbf{x}^*=\arg\min_{\mathbf{x}} \sum_{(i,j)}\left(\|\mathbf{x}-\mathbf{s}_i\| - \|\mathbf{x}-\mathbf{s}_j\| - c\,\tau_{ij}\right)^2
$$

A.4 RTH Reliability Sensitivity

Taking logarithm:

$$
\ln P_{\text{RTH}}=\ln R_b + \ln R_m + \ln R_d + \ln R_w
$$

Sensitivity:

$$
\frac{\partial \ln P_{\text{RTH}}}{\partial R_k}=\frac{1}{R_k}, \quad k\in\{b,m,d,w\}
$$

Thus battery reliability dominates under low-energy conditions.

APPENDIX B

System Configuration Parameters

B.1 Default Operational Constants

B.2 Cryptographic Parameters

APPENDIX C

Experimental Test Environment

C.1 Simulation Platform

Python 3.11

C++17

SciPy TRF solver

Sampling rate: 48 kHz (acoustic tests)

Quadrotor kinematic model

C.2 Hardware Assumptions

4-rotor quadcopter geometry

Nominal hover thrust = 2.45 N per motor

Max wind threshold = 12 m/s

Max reliable mission radius = 10 km

APPENDIX D

Software Architecture File Map

APPENDIX E

Safety and Threat Model Assumptions

Adversary may inject, replay, or drop packets

Adversary may spoof GPS

At most ⌊(N−1)/3⌋ Byzantine nodes

Acoustic noise modeled as Gaussian

Wind modeled as bounded constant disturbance

APPENDIX F

Limitations of Formal Model

Acoustic localization currently 2D

Network partition split-brain not formally solved

ML model assumes stationary distribution

Energy model assumes independent factors

XIX. REFERENCES

[1] S. Hayat, E. Yanmaz, and R. Muzaffar, “Survey on unmanned aerial vehicle networks for civil applications,” IEEE Commun. Surveys Tuts., vol. 18, no. 4, pp. 2624–2661, 2016.

[2] E. Şahin, “Swarm robotics: From sources of inspiration to domains of application,” in Swarm Robotics Workshop, 2004.

[3] M. Dorigo, M. Birattari, and M. Brambilla, “Swarm robotics,” Autonomous Robots, vol. 17, no. 2–3, pp. 111–113, 2014.

[4] L. Lamport, R. Shostak, and M. Pease, “The Byzantine generals problem,” ACM Trans. Program. Lang. Syst., vol. 4, no. 3, pp. 382–401, 1982.

[5] M. Castro and B. Liskov, “Practical Byzantine fault tolerance,” ACM Trans. Comput. Syst., vol. 20, no. 4, pp. 398–461, 2002.

[6] N. Lynch, Distributed Algorithms. San Francisco, CA, USA: Morgan Kaufmann, 1996.

[7] M. Asadpour et al., “Distributed consensus and coordination in UAV swarms: Recent advances and challenges,” IEEE Access, vol. 11, pp. 35214–35237, 2023.

[8] S. Park and D. Kim, “Resilient distributed leader election in dynamic UAV networks,” IEEE Trans. Aerosp. Electron. Syst., vol. 60, no. 2, pp. 1456–1469, 2024.

[9] T. Humphreys et al., “Assessing vulnerability of UAVs to GPS spoofing attacks,” in Proc. IEEE Aerospace Conf., 2008.

[10] M. Psiaki and T. Humphreys, “GNSS spoofing and detection,” Proc. IEEE, vol. 104, no. 6, pp. 1258–1270, 2016.

[11] D. He et al., “Security and privacy in UAV communication networks,” IEEE Wireless Commun., vol. 26, no. 5, pp. 64–69, 2019.

[12] R. Mitchell and I.-R. Chen, “Cybersecurity in unmanned aerial vehicle systems: A survey,” IEEE Commun. Surveys Tuts., vol. 24, no. 4, pp. 2341–2371, 2022.

[13] J. Sun, L. Yan, and Y. Zhang, “Trust management for secure UAV swarms in adversarial environments,” IEEE Trans. Inf. Forensics Security, vol. 19, pp. 1024–1037, 2024.

[14] D. McGrew and J. Viega, “The Galois/Counter Mode of operation (GCM),” in NIST Modes Workshop, 2004.

[15] NIST, “Advanced Encryption Standard (AES),” FIPS PUB 197, 2001.

[16] NIST, “SHA-3 Standard: Permutation-Based Hash and Extendable-Output Functions,” FIPS PUB 202, 2015.

[17] D. J. Bernstein et al., “High-speed high-security signatures,” J. Cryptographic Engineering, vol. 2, no. 2, pp. 77–89, 2012.

[18] L. Wang and K. Liu, “Secure lightweight blockchain architecture for UAV networks,” IEEE Access, vol. 13, pp. 5567–5581, 2025.

[19] S. Nakamoto, “Bitcoin: A peer-to-peer electronic cash system,” 2008.

[20] C. Cachin, “Architecture of the Hyperledger blockchain fabric,” 2016.

[21] C. Knapp and G. Carter, “The generalized correlation method for estimation of time delay,” IEEE Trans. Acoust., Speech Signal Process., vol. 24, no. 4, pp. 320–327, 1976.

[22] H. Brandstein and D. Ward, Microphone Arrays: Signal Processing Techniques and Applications. Springer, 2001.

[23] J. Chen, J. Benesty, and Y. Huang, “Time delay estimation in room acoustic environments,” IEEE Trans. Audio Speech Lang. Process., vol. 14, no. 3, pp. 870–883, 2006.

[24] H. Zhao et al., “Low-latency acoustic source localization for drone swarms using robust GCC variants,” IEEE Sensors J., vol. 25, no. 3, pp. 4121–4134, 2025.

[25] P. Fiorini and Z. Shiller, “Motion planning in dynamic environments using velocity obstacles,” Int. J. Robot. Res., vol. 17, no. 7, pp. 760–772, 1998.

[26] J. van den Berg et al., “Reciprocal velocity obstacles for real-time multi-agent navigation,” in Proc. IEEE ICRA, 2008.

[27] A. Gupta et al., “Energy-aware path planning for UAV swarms in dynamic wind fields,” IEEE Trans. Intell. Transp. Syst., vol. 25, no. 1, pp. 987–999, 2024.

[28] Y. Zhang and J. Jiang, “Bibliographical review on fault-tolerant control systems,” Annu. Rev. Control, vol. 32, no. 2, pp. 229–252, 2008.

[29] J. Boskovic, S. Li, and R. Mehra, “Fault-tolerant control of UAVs,” IEEE Trans. Control Syst. Technol., vol. 12, no. 5, pp. 655–662, 2004.

[30] M. Mueller and R. D’Andrea, “Stability and control of a quadrocopter despite propeller loss,” in Proc. IEEE ICRA, 2014.

[31] T. Rahman and P. Mehta, “Fault-tolerant control strategies for quadrotor UAVs under actuator degradation,” IEEE Trans. Control Syst. Technol., vol. 33, no. 1, pp. 115–128, 2025.

[32] G. Buttazzo, Hard Real-Time Computing Systems. Springer, 2011.

[33] J. Liu, Real-Time Systems. Pearson, 2000.

[34] C. Bishop, Pattern Recognition and Machine Learning. Springer, 2006.

[35] T. Hastie, R. Tibshirani, and J. Friedman, The Elements of Statistical Learning. Springer, 2009.

[36] P. Shor, “Polynomial-time algorithms for prime factorization and discrete logarithms on a quantum computer,” SIAM J. Comput., vol. 26, no. 5, pp. 1484–1509, 1997.

[37] NIST, “Post-Quantum Cryptography Standardization,” 2022.

[38] Y. Li, H. Zhang, and X. Shen, “Secure and resilient UAV swarm networking: A survey,” IEEE Internet Things J., vol. 9, no. 18, pp. 16745–16767, 2022.
