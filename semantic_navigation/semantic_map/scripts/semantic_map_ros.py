#!/usr/bin/env python

from abc import ABCMeta, abstractmethod

class SemanticMapROS(object):
	__metaclass__ = ABCMeta

	@abstractmethod
	def service_callback(self, request):
		pass
