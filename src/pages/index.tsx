import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '../components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Read the Book - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function OverviewSection() {
  return (
    <section className="overview-section">
      <div className="container">
        <h2>Book Overview</h2>
        <div className="overview-content">
          <p>
            The Physical AI Book is a comprehensive hands-on guide that bridges the gap between artificial intelligence and the physical world.
            It covers everything from fundamental concepts to advanced implementations in robotics, computer vision, and embodied AI systems.
          </p>
          <p>
            This field manual provides practical, implementation-ready knowledge that enables readers to build real Physical AI systems,
            from simple sensor-actuator interactions to complex autonomous robots capable of perception, decision-making, and action.
          </p>
        </div>
      </div>
    </section>
  );
}

function LearningOutcomesSection() {
  return (
    <section className="learning-section">
      <div className="container">
        <h2>What You'll Master</h2>
        <div className="learning-outcomes-grid">
          <div className="learning-outcome-card">
            <h3>ROS 2 Fundamentals</h3>
            <p>Learn to build robust robotic systems using the Robot Operating System 2, including nodes, topics, services, and actions.</p>
          </div>
          <div className="learning-outcome-card">
            <h3>Computer Vision</h3>
            <p>Master vision algorithms for object detection, tracking, and scene understanding in real-world environments.</p>
          </div>
          <div className="learning-outcome-card">
            <h3>Sensor Integration</h3>
            <p>Integrate various sensors like cameras, LiDAR, IMUs, and other perception systems into cohesive AI solutions.</p>
          </div>
          <div className="learning-outcome-card">
            <h3>Control Systems</h3>
            <p>Design and implement feedback control systems for precise robot movement and manipulation.</p>
          </div>
          <div className="learning-outcome-card">
            <h3>AI Planning</h3>
            <p>Develop intelligent planning and decision-making algorithms for autonomous robot behavior.</p>
          </div>
          <div className="learning-outcome-card">
            <h3>Embodied Intelligence</h3>
            <p>Combine perception, planning, and action to create truly intelligent physical AI systems.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="A hands-on field manual for building and understanding real Physical AI systems">
      <HomepageHeader />
      <main>
        <OverviewSection />
        <LearningOutcomesSection />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}