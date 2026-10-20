"""
The playground: the NiFT stack's `sim_kinematic` launch on fakeros, for the browser (Pyodide) and natively.

    paths        sys.path for the stand-ins and the repo packages
    launch_set   the nodes, parameter sources, remaps and environment of profile:=sim_kinematic
    sim_backend  the repo's KinematicSimNode with BrowserOctane (octane_bridge)
    runner       the deterministic tick loop
    operator     playground_operator: the page's buttons as ROS messages and service calls
    observer     playground_observer: the stack as the page sees it, and the trace recorder
    perception   playground_perception: /obstacles from the page's obstacles (perception option)
    scenario     save, load and URL encoding of a launch plus its command log
    session      the worker protocol (ARCH section 8)
    cli          a headless native run of a test route
"""
