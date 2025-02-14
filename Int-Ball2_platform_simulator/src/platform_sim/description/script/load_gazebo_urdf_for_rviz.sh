#!/bin/bash

sed -e 's@model://@package://description/model/@g' $1
