plugins {
    kotlin("jvm") version "1.5.0"
    id("org.jlleitschuh.gradle.ktlint") version "10.0.0"
    java
    maven
    checkstyle
    id("org.sonarqube") version "3.1.1"
    `maven-publish`
    `java-library`
}

group = "org.valkyrienskies.kronch"
// Determine the version
if (project.hasProperty("CustomReleaseVersion")) {
    version = project.property("CustomReleaseVersion") as String
} else {
    // Yes, I know there is a gradle plugin to detect git version.
    // But its made by Palantir 0_0.
    val gitRevisionProcess = Runtime.getRuntime().exec("git rev-parse HEAD", emptyArray(), File("."))
    val processInputStream = gitRevisionProcess.inputStream

    var gitRevision = ""
    while (true) {
        val lastReadByte = processInputStream.read()
        if (lastReadByte == -1) break
        gitRevision += lastReadByte.toChar()
    }
    version = "1.0.0+" + gitRevision.substring(0, 10)
}

repositories {
    mavenCentral()
    maven { setUrl("https://dl.bintray.com/kotlin/kotlin-eap") }
}

dependencies {
    // Kotlin
    implementation(kotlin("stdlib-jdk8"))

    // JOML for Math
    api("org.joml", "joml", "1.10.0")
    api("org.joml", "joml-primitives", "1.10.0")

    // Guava
    implementation("com.google.guava", "guava", "29.0-jre")

    // FastUtil for Fast Primitive Collections
    implementation("it.unimi.dsi", "fastutil", "8.2.1")

    // Junit 5 for Unit Testing
    testImplementation("org.junit.jupiter", "junit-jupiter", "5.4.2")

    // LWJGL for Test Suites
    // LWJGL platforms are defined here https://github.com/LWJGL/lwjgl3/blob/161ca91dd9289fd7c1da4f420f10ff472f8671c7/build.gradle.kts#L61-L74
    val lwjglPlatforms = arrayOf(
        null,
        "natives-linux",
        "natives-linux-arm64",
        "natives-linux-arm32",
        "natives-macos",
        // "natives-macos-arm64", // Rip
        "natives-windows",
        "natives-windows-x86"
        // "natives-windows-arm64" // Rip
    )
    val lwjglVersion = "3.2.3"
    for (platform in lwjglPlatforms) {
        testImplementation("org.lwjgl", "lwjgl", version = lwjglVersion, classifier = platform)
        testImplementation("org.lwjgl", "lwjgl-glfw", version = lwjglVersion, classifier = platform)
        testImplementation("org.lwjgl", "lwjgl-opengl", version = lwjglVersion, classifier = platform)
    }
}

tasks.withType<Checkstyle> {
    reports {
        // Do not output html reports
        html.isEnabled = false
        // Output xml reports
        xml.isEnabled = true
    }
}

checkstyle {
    toolVersion = "8.41"
    configFile = file("$rootDir/.checkstyle/checkstyle.xml")
    isIgnoreFailures = false
}

ktlint {
    disabledRules.set(setOf("parameter-list-wrapping"))
    reporters {
        reporter(org.jlleitschuh.gradle.ktlint.reporter.ReporterType.CHECKSTYLE)
    }
}

tasks {
    compileKotlin {
        kotlinOptions.jvmTarget = "1.8"
    }
    compileTestKotlin {
        kotlinOptions.jvmTarget = "1.8"
    }
    compileJava {
        sourceCompatibility = "1.8"
        targetCompatibility = "1.8"
    }
    compileTestJava {
        sourceCompatibility = "1.8"
        targetCompatibility = "1.8"
    }
    test {
        useJUnitPlatform()
        testLogging {
            events("passed", "skipped", "failed")
        }
    }
}

publishing {
    repositories {
        val ghpUser = (project.findProperty("gpr.user") ?: System.getenv("USERNAME")) as String?
        val ghpPassword = (project.findProperty("gpr.key") ?: System.getenv("TOKEN")) as String?
        // Publish to Github Packages
        if (ghpUser != null && ghpPassword != null) {
            maven {
                name = "GithubPackages"
                url = uri("https://maven.pkg.github.com/ValkyrienSkies/Kronch-Physics-Engine")
                credentials {
                    username = ghpUser
                    password = ghpPassword
                }
            }
        }

        // Publish to VS maven
        if ((System.getenv("VS_MAVEN_USERNAME") != null) and (System.getenv("VS_MAVEN_PASSWORD") != null)) {
            maven {
                name = "ValkyrienSkiesMaven"
                url = uri("https://maven.valkyrienskies.org/repository/internal/")
                credentials {
                    username = System.getenv("VS_MAVEN_USERNAME")
                    password = System.getenv("VS_MAVEN_PASSWORD")
                }
            }
        }
    }
    publishing {
        publications {
            create<MavenPublication>("maven") {
                groupId = "org.valkyrienskies"
                artifactId = "kronch"
                version = project.version as String

                from(components["java"])
            }
        }
    }
}

sonarqube {
    properties {
        property("sonar.projectKey", "ValkyrienSkies_kronch")
        property("sonar.organization", "valkyrienskies")
        property("sonar.host.url", "https://sonarcloud.io")
    }
}
