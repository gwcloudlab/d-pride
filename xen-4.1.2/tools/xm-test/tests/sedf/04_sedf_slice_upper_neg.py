#!/usr/bin/python

# Copyright (C) International Business Machines Corp., 2005
# Author: Dan Smith <danms@us.ibm.com>
# Author: Ryan Harper <ryanh@us.ibm.com>

from XmTestLib import *

def get_sedf_params(domain):
    status, output = traceCommand("xm sched-sedf %s" %(domain.getName()))
    return (status, output.split('\n')[1].split())


domain = XmTestDomain(extraConfig = {"sched":"sedf"})

try:
    domain.start(noConsole=True)
except DomainError, e:
    if verbose:
        print "Failed to create test domain because:"
        print e.extra
    FAIL(str(e))

# get current param values as baseline
(status, params) = get_sedf_params(domain)

# check rv
if status != 0:
    FAIL("Getting sedf parameters return non-zero rv (%d)", status)

# parse out current params
(name, domid, p, s, l, e, w) = params

# set slice > than current period
slice  = str(float(p)+1)

opts = "%s -s %s" %(domain.getName(), slice)
(status, output) = traceCommand("xm sched-sedf %s" %(opts))

# we should see this output from xm 
eyecatcher = "Failed to set sedf parameters"

# check for failure
if output.find(eyecatcher) >= 0:
    FAIL("sched-sedf let me set a slice bigger than my period.")

# Stop the domain (nice shutdown)
domain.stop()
